import os
import math
import vl53l0x
from random import randint
from machine import Pin, SoftI2C, UART
from time import sleep_ms

'''piece variable'''
# Detection
MAX_MESURE = 2000

# Worm motion
HEAD_PERIMETER = 150
F_TRACKING = 1000 # in mm/min
F_STRESS = 2000 # in mm/min

# Soft limits
RAIL_MIN = 10
RAIL_MAX = 200

'''init'''
# variable
pos = [0,0,0,0] #x,y,a,b
prev_pos = pos 
closer_body = {"angle":0,"distance":0}

# --TOF--

# Initialize i2c
i2c = SoftI2C(sda=Pin(18), scl=Pin(16))

# Check what's on i2c the bus
# print("Scanning I2C bus...")
# devices = i2c.scan()
# print(f"Found devices: {[hex(d) for d in devices]}")

xshut = [
    Pin(10, Pin.OUT),
    Pin(1, Pin.OUT),
    Pin(2, Pin.OUT),
    Pin(3, Pin.OUT),
    Pin(4, Pin.OUT),
    Pin(5, Pin.OUT),
    Pin(6, Pin.OUT),
    Pin(7, Pin.OUT)
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
uart = UART(1, baudrate=115200, tx=12, rx=13)

# Reset/initialize GRBL
uart.write(b"\r\n\r\n")
sleep_ms(5000) 
uart.flush()

# Unlock GRBL
uart.write(b"$X\n")
sleep_ms(100)

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
    
    # Populate the target value and add a random value within the range of possible to ensure realisitic move
    pos[0] = int(math.cos(closer_body["angle"]) * HEAD_PERIMETER) + randint(-17,17)
    pos[1] = int(math.sin(closer_body["angle"]) * HEAD_PERIMETER) + randint(-17,17)

    # Clamping the value (soft limits)
    pos[0] = min(RAIL_MAX,max(RAIL_MIN,pos[0]))
    pos[1] = min(RAIL_MAX,max(RAIL_MIN,pos[1]))
    
    #tracking state
    feedrate = F_TRACKING
    
    #print(pos)

    # Send coordonate to fluid_nc
    uart.write(f"G1X{pos[0]}Y{pos[1]}F{feedrate}\n".encode())
    
    # Ensure end of execution
    pos_dif = [x - y for x, y in zip(pos, prev_pos)]
    abs_pos_dif = [abs(x) for x in pos_dif]
    max_step = max(abs_pos_dif) # get the larger stepping operation
    t_exec = max_step / (feedrate/60) * 1000 # in ms
    
    #save prev_pos_value
    prev_pos = pos.copy()
    
    sleep_ms(int(t_exec))
    
    

