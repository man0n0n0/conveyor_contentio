import os
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C  
from time import sleep_ms
from stepper import Stepper

'''piece variable'''
rail_offset = 100
rail_length = 8000
speed = 1000 # steps/sec
steps_per_mm = 40 # for 20 teeth GT2 pulley

# --- SCREEN INIT ---
i2c = I2C(0, sda=Pin(5), scl=Pin(6))

display = SSD1306_I2C(72, 40, i2c)

def display_txt(t):
    display.fill(0)
    display.text("[CONVE0R]",0, 0, 1)
    display.text(t,0, 15, 1)
    display.text("[CONVE0R]",0, 30, 1)
    display.show()

# --- STEPPER INIT ---
s = Stepper(1,0,2) #stp,dir,en
#create an input pin for the end switches (switch connects pin to GND)
end_s = [Pin(3, Pin.IN),Pin(4, Pin.IN)] #pull down doest work properly

# --- HOMING ---
def homing():
    display_txt(f" homing")
    s.speed(speed) 
    s.free_run(-1) #move forward

    while end_s[0].value() == 0:
        pass

    s.stop() #stop as soon as the switch is triggered
    s.overwrite_pos(0) #set position as 0 point
    s.target(0) #set the target to the same value to avoid unwanted movement
    
    display_txt(f"to offset")
    
    s.track_target()
    s.target(rail_offset*steps_per_mm)
    sleep_ms(5000)
        
    display_txt(f" homed")
    sleep_ms(1000)
    
# --- MOVE BETWEEN ENDS ---    
def move_to(pos_mm):
    display_txt(f"{pos_mm}")
    target_steps = pos_mm * steps_per_mm
    s.target(target_steps)
    s.track_target()
    while s.get_pos() != target_steps:
        if end_s[0].value() or end_s[1].value():
            homing()
            return
        sleep_ms(10)

'''execution'''
homing()

while True :
    move_to(rail_length-rail_offset)   # move to far end
    sleep_ms(500)
    move_to(rail_offset)             # back to start
    sleep_ms(500)
