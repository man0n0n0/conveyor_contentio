import os
import math
import vl53l0x
import network
import socket
from random import randint
from machine import Pin, SoftI2C
from time import sleep_ms, time

'''piece variable'''
# Detection
MAX_MESURE = 200 #in cm

# Worm motion
THREAD_LENGTH = 90
HEAD_PERIMETER = THREAD_LENGTH//2
ENABLE_DURATION = 2 # in sec
F_TRACKING = 5000 # in mm/min
F_STRESS = 2000 # in mm/min

'''init'''
# variable
pos = [0,0,0,0] #x,y,a,b
prev_pos = pos
prev_min_index = 0
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
] # reversed list order to fit physicalitty of the device

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
sleep_ms(500)

# --stepper control--
SSID = 'FluidNC'
PASSWORD = '12345678'
FLUIDNC_IP = '192.168.0.1'   # Change to your FluidNC IP

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print('Connecting to Wi-Fi...')
    wlan.connect(SSID, PASSWORD)
    for _ in range(20):  # 10 seconds total
        if wlan.isconnected():
            break
        print('.', end='')
        sleep_ms(500)

# Connect to FluidNC via Telnet (port 23)
addr = socket.getaddrinfo(FLUIDNC_IP, 23)[0][-1]
s = socket.socket()
s.connect(addr)
print(f'Connected to {SSID}')

# Wake up FLUIDNC
s.send(b'\r\n') 
sleep_ms(2000)

#s.send(b'$H\n')
#sleep_ms(5000)

s.send(b'$X\n')
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
        closer_body["angle"] = min_index*(math.pi/4) # 45 deg in radian
        closer_body["distance"] = min_range
        print(closer_body,min_index*45)
    
    if min_index != prev_min_index :
        
        if IDLE :
            s.send(b"$Motor/Enable\n") #enable all stepper
            sleep_ms(10)
            IDLE = False
                        
        # Populate the target value and add a random value within the range of possible to ensure realisitic move
        pos[0] = int(math.cos(closer_body["angle"]) * HEAD_PERIMETER) + randint(-17,17)
        pos[1] = int(math.sin(closer_body["angle"]) * HEAD_PERIMETER) + randint(-17,17)

        #tracking state
        feedrate = F_TRACKING
        
        print(pos)

        # Send coordonate to fluid_nc
        # Send G-code
        s.send(f"G1X{pos[0]}Y{pos[1]}F{feedrate}\n".encode())
        
        # Ensure end of execution
        pos_dif = [x - y for x, y in zip(pos, prev_pos)]
        abs_pos_dif = [abs(x) for x in pos_dif]
        max_step = max(abs_pos_dif) # get the larger stepping operation
        t_exec = max_step / (feedrate/60) * 1000 # in ms
        
        #save prev_pos_value
        prev_pos = pos.copy()
        prev_min_index = min_index
        prev_move_time = time()
        
        sleep_ms(int(t_exec))
    
    else:
        if time() - prev_move_time < ENABLE_DURATION :
            s.send(b"$Motor/Disable\n") #disable all stepper
            IDLE = True
        sleep_ms(50) # waiting time between readings
    
    

