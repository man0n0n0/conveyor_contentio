import os
import vl53l0x
from machine import Pin, SoftI2C
from time import sleep_ms

'''piece variable'''

'''init'''
i2c = SoftI2C(sda=Pin(18), scl=Pin(16))
# Check what's on the bus
# print("Scanning I2C bus...")
# devices = i2c.scan()
# print(f"Found devices: {[hex(d) for d in devices]}")
# '''--TOF'''
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
closer_body = {"angle"=0,"distance"=0}

for index , power_pin in enumerate(xshut): 
    print(power_pin)
    power_pin.value(1)
    sleep_ms(50)  # Give sensor time to boot up
    
    vl53.insert(index , vl53l0x.VL53L0X(i2c))  
    if index < len(xshut) - 1:
        vl53[index].set_address(index + 0x30)

'''--stepper'''
#todo

'''execution'''
while True:

    # Read all sensor values and find the minimum
    min_range = float('inf')
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
        closer_body["angle"] = min_index*45 
        closer_body["distance"] = min_range
        
    else:
        print("No valid readings")
    
    
    # Send coordonate to fluid_nc
    
    
    
    
    sleep_ms(10)  # Small delay between readings
    
    

