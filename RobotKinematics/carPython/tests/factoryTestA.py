#  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
#| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/ 
# Osoyoo Raspberry Pi V2.1 car Lesson 1 tutorial
# tutorial url: https://osoyoo.com/?p=49432
# converted to Harker car: M Baynes


import time
# Import the PCA9685 module.
from board import SCL, SDA
import busio
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm =  PCA9685(i2c_bus)

move_speed = 0x7FFF  # half of Max pulse length out of 0xFFFF

# Set frequency to 60hz, good for servos.
pwm.frequency = 60
GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
GPIO.setwarnings(False)
#define L298N(Model-Pi motor drive board) GPIO pins
IN1_FRONT = 1  #FRONT Right  motor(BK1 port) direction PCA9685 port 1
IN2_FRONT = 2  #FRONT Right  motor(BK1 port) direction PCA9685 port 2
IN3_FRONT = 5  #FRONT Left motor(BK3 port) direction PCA9685 port 5
IN4_FRONT = 6  #FRONT Left motor(BK3 port) direction PCA9685 port 6
ENA_FRONT = 0 	#ENA_FRONT _B area Left motor speed PCA9685 port 0
ENB_FRONT = 4	#ENB_FRONT _B area Right motor speed PCA9685 port 4

IN1_REAR = 9  #Rear Right  motor(AK1 port) direction PCA9685 port 9
IN2_REAR = 10  #Rear Right  motor(AK1 port) direction PCA9685 port 10
IN3_REAR = 13  #Rear Left motor(AK3 port) direction PCA9685 port 13
IN4_REAR = 14  #Rear Left motor(AK3 port) direction PCA9685 port 14
ENA_REAR = 8 	#ENA_REAR _A area Left motor speed PCA9685 port 8
ENB_REAR = 12	#ENB_REAR _A area Right motor speed PCA9685 port 12

# for IN1, IN2, IN3, IN4 define 1  and 0 settings
high = 0xFFFF #1 
low  = 0      #0 

def changespeed(speed):
    pwm.channels[ENA_FRONT].duty_cycle = speed
    pwm.channels[ENB_FRONT].duty_cycle = speed
    pwm.channels[ENA_REAR].duty_cycle = speed
    pwm.channels[ENB_REAR].duty_cycle = speed

def stop_car():
    pwm.channels[IN1_FRONT].duty_cycle = low
    pwm.channels[IN2_FRONT].duty_cycle = low
    pwm.channels[IN3_FRONT].duty_cycle = low
    pwm.channels[IN4_FRONT].duty_cycle = low
    pwm.channels[IN1_REAR].duty_cycle = low
    pwm.channels[IN2_REAR].duty_cycle = low
    pwm.channels[IN3_REAR].duty_cycle = low
    pwm.channels[IN4_REAR].duty_cycle = low
    changespeed(0)
    
def rr_ahead():
    pwm.channels[IN1_REAR].duty_cycle = high
    pwm.channels[IN2_REAR].duty_cycle = low
    #pwm.channels[ENA_REAR].duty_cycle = speed
    
def rr_back():
    pwm.channels[IN1_REAR].duty_cycle = low
    pwm.channels[IN2_REAR].duty_cycle = high
    #pwm.channels[ENA_REAR].duty_cycle = speed
    
def rl_ahead():  
    pwm.channels[IN3_REAR].duty_cycle = high
    pwm.channels[IN4_REAR].duty_cycle = low
    #pwm.channels[ENB_REAR].duty_cycle = speed   

def rl_back():  
    pwm.channels[IN3_REAR].duty_cycle = low
    pwm.channels[IN4_REAR].duty_cycle = high
    #pwm.channels[ENB_REAR].duty_cycle = speed   

def fr_ahead():
    pwm.channels[IN1_FRONT].duty_cycle = high
    pwm.channels[IN2_FRONT].duty_cycle = low
    #pwm.channels[ENA_FRONT].duty_cycle = speed
    
def fr_back():
    pwm.channels[IN1_FRONT].duty_cycle = low
    pwm.channels[IN2_FRONT].duty_cycle = high
    #pwm.channels[ENA_FRONT].duty_cycle = speed
    
def fl_ahead():  
    pwm.channels[IN3_FRONT].duty_cycle = high
    pwm.channels[IN4_FRONT].duty_cycle = low
    #pwm.channels[ENB_FRONT].duty_cycle = speed   

def fl_back():  
    pwm.channels[IN3_FRONT].duty_cycle = low
    pwm.channels[IN4_FRONT].duty_cycle = high
    #pwm.channels[ENB_FRONT].duty_cycle = speed  

def go_ahead(speed):
    rl_ahead()
    rr_ahead()
    fl_ahead()
    fr_ahead()
    changespeed(speed)
    
def go_back(speed):
    rr_back()
    rl_back()
    fr_back()
    fl_back()
    changespeed(speed)

#making right turn   
def turn_right(speed):
    rl_ahead()
    rr_back()
    fl_ahead()
    fr_back()
    changespeed(speed)
      
#make left turn
def turn_left(speed):
    rr_ahead()
    rl_back()
    fr_ahead()
    fl_back()
    changespeed(speed)

# parallel left shift 
def shift_left(speed):
    fr_ahead()
    rr_back()
    rl_ahead()
    fl_back()
    changespeed(speed)
    
# parallel right shift 
def shift_right(speed):
    fr_back()
    rr_ahead()
    rl_back()
    fl_ahead()
    changespeed(speed)
      
def upper_right(speed):
    rr_ahead()
    fl_ahead()
    changespeed(speed)

def lower_left(speed):
    rr_back()
    fl_back()
    changespeed(speed)
    
def upper_left(speed):
    fr_ahead()
    rl_ahead()
    changespeed(speed)
    
def lower_right(speed):
    fr_back()
    rl_back()
    changespeed(speed)
    
    
print('ahead...')
go_ahead(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('backward...')
go_back(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('turn left...')
turn_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('turn right...')
turn_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('shift right...')
shift_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('shift left...')
shift_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('diagonal left fwd...')
upper_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('diagonal right back...')
lower_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('diagonal right fwd...')
upper_right(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('diagonal left back...')
lower_left(move_speed)
time.sleep(1)
stop_car()
time.sleep(0.5)

print('press Ctrl-C to quit...')
GPIO.cleanup()   