import network, time
from machine import Pin, SoftI2C
 

i2c = SoftI2C(sda=Pin(18), scl=Pin(21))


sta = network.WLAN(network.STA_IF)
sta.active(False)
time.sleep(1)
sta.active(True)
time.sleep(1)
print("Active:", sta.active())
print("Scanning...")
print(sta.scan())
