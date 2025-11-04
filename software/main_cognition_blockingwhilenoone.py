import os
import math
import vl53l0x
from random import randint, choice
from machine import Pin, SoftI2C, UART
from time import sleep_ms, time

'''piece variable'''
# Detection
MAX_MESURE = 2000 #in cm

# Worm motion
THREAD_LENGTH = 110
HEAD_PERIMETER = THREAD_LENGTH//2

ENABLE_DURATION = 120 # in sec

F_TRACKING = 3500 # in mm/min
F_TRACKING_INTERVAL = 500

F_STRESS = 12500 # in mm/min
F_STRESS_INTERVAL = 250
PRESENCE_DURATION_OFFSET = 3 # time before activating stess_mode 
STRESS_DURATION = 5
STRESS_MACRO = "t.g"

SMOOTH_STEPS = 10  # number of interpolated steps per move

# Breathing behavior
BREATHING_INTERVAL = 30  # how far to move during breathing
BREATHING_INCREMENT = 10
F_BREATHING = 500
BREATHING_COUNT = 100 # number of cycle before moving

'''init'''
# variable
pos = [0,0,0,0] #x,y,a,b
prev_pos = pos
prev_min_index = 0
cycle_count = 0
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
# Initialize UART
uart = UART(1, baudrate=115200, tx=13, rx=12)

# Wake up FLUIDNC
uart.write(b'\r\n') 
sleep_ms(2000)

uart.write(b'$H\n')
sleep_ms(15000)

uart.write(b'$X\n')
sleep_ms(100)

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
            if range_val < min_range:
                min_range = range_val
                min_index = i
        except:
            pass  # Skip sensor if reading fails
    
    if min_index != -1:
        closer_body["angle"] = min_index*(math.pi/4) #in radian
        closer_body["distance"] = min_range
        last_presence_reading = time()
    
    if min_index != prev_min_index :
        
        if IDLE :
            uart.write(b"$Motor/Enable\n") #enable all stepper
            sleep_ms(50)
            IDLE = False
                        
                # --- Compute new target position ---
        pos = [
            int(math.cos(closer_body["angle"]) * HEAD_PERIMETER),
            int(math.sin(closer_body["angle"]) * HEAD_PERIMETER),
            0,
            0
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
                int(prev_pos[1] + pos_dif[1] * ease_t)
            ]

            # Send intermediate G-code command
            uart.write(f"G1X{intermediate_pos[0]}Y{intermediate_pos[1]}F{feedrate}\n".encode())

            # Wait proportional to execution time
            sleep_ms(int(substep_time))

        # --- Save state for next iteration ---
        prev_pos = pos.copy()
        prev_move_time = time()
        prev_min_index = min_index

    else:
        
        if PRESENCE_DURATION_OFFSET < time() - prev_move_time < PRESENCE_DURATION_OFFSET + STRESS_DURATION : 
            
            # --- Compute new target position ---
            pos = [
                choice([-HEAD_PERIMETER, HEAD_PERIMETER]),
                choice([-HEAD_PERIMETER, HEAD_PERIMETER]),
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


        elif time() - prev_move_time > ENABLE_DURATION :
            # slow breathing beaviour
            if cycle_count % BREATHING_COUNT == 0:

                uart.write(b"$Motor/Enable\n") # temporary enabling

                # Compute intermediate coordinates with a clamp system 
                pos = [
                    max(-BREATHING_INTERVAL, min(BREATHING_INTERVAL, int(prev_pos[0] + randint(-BREATHING_INCREMENT,BREATHING_INCREMENT)))),
                    max(-BREATHING_INTERVAL, min(BREATHING_INTERVAL, int(prev_pos[1] + randint(-BREATHING_INCREMENT,BREATHING_INCREMENT)))),
                    max(-BREATHING_INTERVAL, min(BREATHING_INTERVAL, int(prev_pos[2] + randint(-BREATHING_INCREMENT,BREATHING_INCREMENT)))),
                    max(-BREATHING_INTERVAL, min(BREATHING_INTERVAL, int(prev_pos[3] + randint(-BREATHING_INCREMENT,BREATHING_INCREMENT))))
                ]

                # Send intermediate G-code command
                uart.write(f"G1X{pos[0]}Y{pos[1]}A{pos[2]}B{pos[3]}F{F_BREATHING}\n".encode())

                prev_pos = pos.copy()
                
                t_exec = 5 / (F_BREATHING / 60) * 1000  # ms 5 mm move
                sleep_ms(int(t_exec)) # waiting time between readings

            uart.write(b"$Motor/Disable\n") #disable all stepper
            IDLE = True
               
        cycle_count += 1
        sleep_ms(20)
    

