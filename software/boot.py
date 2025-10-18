# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()
####
# BOOT FILE FOR esp32 c3 supermini
####
from time import sleep_ms
from machine import PWM, Pin, I2C
 
led = Pin(8, Pin.OUT)
b_boot = Pin(9, Pin.IN)

#blink
led.value(1)
sleep_ms(100)
led.value(0)
sleep_ms(100)

#blink
led.value(1)
sleep_ms(100)
led.value(0)
sleep_ms(100)

#blink
led.value(1)
sleep_ms(100)
led.value(0)
sleep_ms(100)

if b_boot.value() == 0 :
    pass

else :
    import main_carriage

