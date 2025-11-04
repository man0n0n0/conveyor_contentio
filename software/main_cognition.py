import os
import math
import vl53l0x
from random import randint, choice
from machine import Pin, SoftI2C, UART
from time import sleep_ms, time, ticks_ms, ticks_diff

'''debug'''
debug = False
print(f"DEBUG STATE : {debug}")

'''piece variable'''
# Security
HOMING_COUNT = 25 #number of stress cycle

# Detection
MIN_MESURE = 300 # in mm
MAX_MESURE = 2000 #in mm

# Worm motion
THREAD_LENGTH = 110
HEAD_PERIMETER = THREAD_LENGTH//2

ENABLE_DURATION = 120 # in sec

TRACKING_VARIANCE = 5 #in mm
F_TRACKING = 2000 # in mm/min
F_TRACKING_INTERVAL = 500 #random variability
TRACKING_DURATION = 6

F_STRESS = 10000 # in mm/min
F_STRESS_INTERVAL = 250 # random variability
STRESS_OFFSET = 10 # to reduce travel (reduce system usage) === NOT USED
STRESS_TRIGER_DISTANCE = 600 #in cm
STRESS_DURATION = 5

SMOOTH_STEPS = 10  # number of interpolated steps per move

# Breathing behavior (going toward the center of the actuators for releaving)
BREATHING_INTERVAL = 30  # interval of mouvement for breathing
BREATHING_INCREMENT = 12 # deplacmeent per cycle
F_BREATHING = 500
BREATHING_WAIT = 3 # waiting between movement

'''init'''
# variable
pos = [0,0,0,0] #x,y,a,b
directions=[1,1,1,1] # positiv cause the homign is negative
prev_pos = pos
prev_min_index = 0
cycle_count = 0
stress_cycle_count = 0
prev_move_time = time()
closer_body = {"angle":0,"distance":0}
IDLE = False

# --TOF--
# Initialize i2c
i2c = SoftI2C(sda=Pin(18), scl=Pin(21))

# Check what's on i2c the bus
# print("Scanning I2C bus...")
# devices = i2c.scan()
# print(f"Found devices: {[hex(d) for d in devices]}")

xshut = [
    Pin(4, Pin.OUT),
    Pin(3, Pin.OUT),   
    Pin(2, Pin.OUT),    
    Pin(1, Pin.OUT),    
    Pin(10, Pin.OUT),
    Pin(7, Pin.OUT),
    Pin(6, Pin.OUT),
    Pin(5, Pin.OUT)
]

for power_pin in xshut:
    power_pin.value(0)
    
sleep_ms(100)  # Give time for sensors to power down

vl53 = []

for index , power_pin in enumerate(xshut): 
    print(power_pin)
    power_pin.value(1)
    sleep_ms(50)  # Give sensor time to boot up
    
    vl53.insert(index , vl53l0x.VL53L0X(i2c))  
    if index < len(xshut) - 1:
        vl53[index].set_address(index + 0x30)

# --stepper control--

def homing(timeout_ms=60000):
    """Send $H to FluidNC and wait until homing is complete (status: Idle)."""
    if debug:
        print("homing")
        
    sleep_ms(500) # to ensure there is no fantom message
    
    uart.read()  # clear input buffer
    uart.write(b"$H\n")
    
    start = ticks_ms()
    buffer = b""

    while True:
        uart.write(b"?\n")
        sleep_ms(50)
        
        while uart.any():
            line = uart.readline()
            if not line:
                continue
            if b"<Idle" in line:
                return True
            if b"<Alarm" in line:
                return False
            if ticks_diff(ticks_ms(), start) > timeout_ms:
                return False
        sleep_ms(20)
        
def send_gcode_safe(cmd, timeout_ms=10000):
    """Send one G-code line and wait until FluidNC is Idle before returning."""
    uart.read()  # clear input buffer
    uart.write((cmd + "\n").encode())

    start = ticks_ms()
    buffer = b""

    while True:
        uart.write(b"?\n")
        sleep_ms(50)
        
        while uart.any():
            line = uart.readline()
            if not line:
                continue
            if b"<Idle" in line:
                return True
            if b"<Alarm" in line:
                return False
            if ticks_diff(ticks_ms(), start) > timeout_ms:
                return False
        sleep_ms(20)
        
# Initialize UART
uart = UART(1, baudrate=115200, tx=13, rx=12)

# Wake up FLUIDNC
sleep_ms(1000)
uart.write(b'\r\n') 
sleep_ms(2000)

homing()

uart.write(b'$X\n')
sleep_ms(500)

uart.write(b'G0X0Y0A0B0\n') #put back to the center
sleep_ms(1000)

''''execution'''
while True:
    # Read all sensor values and find the minimum
    min_range = MAX_MESURE
    min_index = -1
    
    for i, sensor in enumerate(vl53):
        try:
            range_val = vl53[i].range
            if MIN_MESURE < range_val < min_range:
                min_range = range_val
                min_index = i
        except:
            pass  # Skip sensor if reading fails
    
    if min_index != -1 or prev_move_time < TRACKING_DURATION :
        # if presency is detected (during the last TRACKING DURATION secondes)
        closer_body["angle"] = min_index*(math.pi/4) #in radian
        closer_body["distance"] = min_range
        last_presence_reading = time()
        
        if debug:
            print("tracking during {TRACKING_DURATION} : {closer_body}")
    
        if IDLE :
            uart.write(b"$Motor/Enable\n") #enable all stepper
            sleep_ms(50)
            IDLE = False
            
            
        if closer_body["distance"] < STRESS_TRIGER_DISTANCE :
        # STRESS BEAVIOUR
            if debug:
                print("stress")
                
            # --- Compute new target position ---
            if prev_pos[0] <= 0 :
                x = HEAD_PERIMETER-STRESS_OFFSET
            else :
                x = -HEAD_PERIMETER+STRESS_OFFSET
                
            pos = [
                x,
                x*-1,
                randint(-HEAD_PERIMETER, HEAD_PERIMETER),
                randint(-HEAD_PERIMETER, HEAD_PERIMETER)
            ]

            # --- Random feedrate to simulate realistic motion ---
            feedrate = randint(F_STRESS - F_STRESS_INTERVAL, F_STRESS + F_STRESS_INTERVAL)

            # --- Compute distance and estimated execution time (in ms) ---
            pos_dif = [x - y for x, y in zip(pos, prev_pos)]
            abs_pos_dif = [abs(x) for x in pos_dif]
            max_step = max(abs_pos_dif)
            t_exec = max_step / (feedrate / 60) * 1000  # ms total move time

            # --- Interpolated smooth movement ---
            dx = pos_dif[0] / SMOOTH_STEPS
            dy = pos_dif[1] / SMOOTH_STEPS

            # Split total time evenly across substeps
            substep_time = t_exec / SMOOTH_STEPS

            for step in range(1, SMOOTH_STEPS + 1):
                # Optional: cosine easing for smoother acceleration/deceleration
                t = step / SMOOTH_STEPS
                ease_t = (1 - math.cos(t * math.pi)) / 2  # smoothstep curve

                # Compute intermediate coordinates
                intermediate_pos = [
                    int(prev_pos[0] + pos_dif[0] * ease_t),
                    int(prev_pos[1] + pos_dif[1] * ease_t),
                    int(prev_pos[2] + pos_dif[2] * ease_t),
                    int(prev_pos[3] + pos_dif[3] * ease_t)
                ]

                # Send intermediate G-code command
                uart.write(f"G1X{intermediate_pos[0]}Y{intermediate_pos[1]}A{intermediate_pos[2]}B{intermediate_pos[3]}F{feedrate}\n".encode())

                # Wait proportional to execution time
                sleep_ms(int(substep_time))

            # --- Save state for next iteration ---
            prev_pos = pos.copy()
            
            # --- increment the counter to ensure a proper homing state of the device
            stress_cycle_count += 1
                
        else :
        # TRACKING BEAHAVIOUR
            if debug :
                print("tracking")
                    # --- Compute new target position : including variance ---
            pos = [
                int(math.cos(closer_body["angle"]) * HEAD_PERIMETER)+randint(-TRACKING_VARIANCE,TRACKING_VARIANCE),
                int(math.sin(closer_body["angle"]) * HEAD_PERIMETER)+randint(-TRACKING_VARIANCE,TRACKING_VARIANCE),
                max(-HEAD_PERIMETER,min(HEAD_PERIMETER,prev_pos[2]+randint(-TRACKING_VARIANCE,TRACKING_VARIANCE))),
                max(-HEAD_PERIMETER,min(HEAD_PERIMETER,prev_pos[3]+randint(-TRACKING_VARIANCE,TRACKING_VARIANCE)))
            ]

            # --- Random feedrate to simulate realistic motion ---
            feedrate = randint(F_TRACKING - F_TRACKING_INTERVAL, F_TRACKING + F_TRACKING_INTERVAL)

            # --- Compute distance and estimated execution time (in ms) ---
            pos_dif = [x - y for x, y in zip(pos, prev_pos)]
            abs_pos_dif = [abs(x) for x in pos_dif]
            max_step = max(abs_pos_dif)
            t_exec = max_step / (feedrate / 60) * 1000  # ms total move time

            # --- Interpolated smooth movement ---
            dx = pos_dif[0] / SMOOTH_STEPS
            dy = pos_dif[1] / SMOOTH_STEPS

            # Split total time evenly across substeps
            substep_time = t_exec / SMOOTH_STEPS

            for step in range(1, SMOOTH_STEPS + 1):
                # Optional: cosine easing for smoother acceleration/deceleration
                t = step / SMOOTH_STEPS
                ease_t = (1 - math.cos(t * math.pi)) / 2  # smoothstep curve

                # Compute intermediate coordinates
                intermediate_pos = [
                    int(prev_pos[0] + pos_dif[0] * ease_t),
                    int(prev_pos[1] + pos_dif[1] * ease_t),
                    int(prev_pos[2] + pos_dif[2] * ease_t),
                    int(prev_pos[3] + pos_dif[3] * ease_t)
                ]

                # Send intermediate G-code command
                uart.write(f"G1X{intermediate_pos[0]}Y{intermediate_pos[1]}A{intermediate_pos[2]}B{intermediate_pos[3]}F{feedrate}\n".encode())

                # Wait proportional to execution time
                sleep_ms(int(substep_time))

            # --- Save state for next iteration ---
            prev_pos = pos.copy()
            prev_move_time = time()
            prev_min_index = min_index

    else:
        # no detection
        if debug:
            print(f"no detection")
        
        if stress_cycle_count - HOMING_COUNT > 0:
            homing()
            stress_cycle_count = 0
            
        if time()-prev_move_time > BREATHING_WAIT:
            # slow breathing beaviour = going back to center
            if IDLE :
                uart.write(b"$Motor/Enable\n") # temporary enabling

            # Compute oscillating coordinates
            pos = []
            
            for i, p in enumerate(prev_pos):
                if p >= BREATHING_INTERVAL:
                    directions[i] = -1
                elif p <= -BREATHING_INTERVAL:
                    directions[i] = 1
                    
                # clamping mecanism
                p = max(-HEAD_PERIMETER,min(HEAD_PERIMETER,p + (directions[i] * BREATHING_INCREMENT)))
                
                pos.append(p)

            # Send intermediate G-code command
            send_gcode_safe(f"G1X{pos[0]}Y{pos[1]}A{pos[2]}B{pos[3]}F{F_BREATHING}")

            prev_pos = pos.copy()
            prev_move_time = time()

            uart.write(b"$Motor/Disable\n") #disable all stepper
            IDLE = True
               
        cycle_count += 1
        sleep_ms(50)
    

