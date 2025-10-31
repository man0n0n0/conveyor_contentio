# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()
####
# BOOT FILE FOR esp32 s2 mini
####
from time import sleep_ms
from machine import PWM, Pin, I2C
 
led = Pin(15, Pin.OUT)
b_boot = Pin(0, Pin.IN)

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
    import main_cognition
    

