import time #used to set delay time to control moving distance
#set up Raspberry Pi
import RPi.GPIO as GPIO #control through GPIO pins not BCM

#set up PC9685 osoyoo/AdaFruit
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685

#pins
PWMOEN = 7 #PCA9685 OEn pin
# front controller
ENAFR = 0
IN1FR = 1
IN2FR = 2

IN3FL = 5
IN4FL = 6
ENBFL = 4
# rear controller
ENARR = 8
IN1RR = 9
IN2RR = 10

IN3RL = 13
IN4RL = 14
ENBRL = 12
 
# GPIO.setup(outputPIN, GPIO.OUT) 
# GPIO.setup(inputPIN, GPIO.IN) ???
#PWM RPi 4 has two channels, but each can be used twice
#PWM channel 0 on pin 32 = pin 12; PWM channel 0 on pin 33 = pin 35
#PWM create & start each pin
# pwm = GPIO.PWM(pwmPIN,1000)  creates instance, where 1000 is the base frequency
#channel alternate, uses same frequency (or overrides for both channels)
# pwm.start(speed)             starts and sets speed, where speed can be >= 0 up to 4095
#channel alternate, uses same frequency (or overrides for both channels)

# create i2c bus interface to access PCA9685, for example
i2c = busio.I2C(SCL, SDA)    #busio.I2C(board.SCL, board.SDA) create i2c bus 
#i2c = busio.I2C(3,2) #if board not available
pca = PCA9685(i2c)           #adafruit_pca9685.PCA9685(i2c)   instance PCA9685 on bus
pca.frequency = 60 #set pwm clock in Hz (debug was 1000)
# usage: pwm_channel = pca.channels[0] instance example
#        pwm_channel.duty_cycle = speed (0 .. 100)  speed example
pwmOEn = GPIO.setup(PWMOEN, GPIO.OUT) 

#equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop): 
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

#for 0 to 100, % speed as integer, to use for PWM 
#full range 0xFFFF, but PCS9685 ignores last Hex digit as only 12 bit resolution)
def getPWMPer(value): 
  return int(valmap(value, 0, 100, 0, 0xFFFF))

# for IN1, IN2, define 1  and 0 settings
high = 0xFFFF #1 was True
low  = 0      #0 was False

class Wheel:
  def __init__(self, name, enCh,in1Ch,in2Ch):
    self.name =name #for debug
    self.en  = pca.channels[enCh]  #EN  wheel 'speed', actually sets power
    self.in1 = pca.channels[in1Ch] #IN1, IN3 wheel direction control 1
    self.in2 = pca.channels[in2Ch] #IN2, IN4 wheel direction control 2
    #If IN1=True  and IN2=False motor moves forward, 
    #If IN1=False and IN2=True  motor moves backward
    #in both other cases motor will stop/brake
    #right: ENA, IN1, IN2 
    #left:  ENB, IN3, IN4 - IN1=IN4 and IN2=IN3 but motor is reversed, so swap 

    #print("created Wheel "+str(self.name)+" at "+str(enCh)+" "+str(in1Ch)+" "+str(in2Ch)) #debug

  def move(self,speed):
  	self.in1.duty_cycle = high if speed > 0 else low
  	self.in2.duty_cycle = low  if speed > 0 else high
  	self.en.duty_cycle  = getPWMPer(speed) if speed > 0 else getPWMPer(-speed)
  	#print("move "+self.name+" @ "+str(speed)) #debug
  	#positive speed forward, negative speed reverse/back; 0 coast, 100% = 4095
  	
  def brake(self):
  	self.in1.duty_cycle = low
  	self.in2.duty_cycle = low
  	#print("brake "+self.name) #debug
  	#electric braking effect, should stop movement
  	
#end of Wheel class
  
#Set up Wheel instances with connections, ch 0 is left end, 
#leaving one pin per quad for future
rl = Wheel("rl", ENBRL, IN3RL, IN4RL) #Rear-left wheel
rr = Wheel("rr", ENARR, IN1RR, IN2RR) #Rear-right wheel
fl = Wheel("fl", ENBFL, IN3FL, IN4FL) #Front-left wheel
fr = Wheel("fr", ENAFR, IN1FR, IN2FR) #Front-right wheel
 
#Movement control examples   
#rear right motor moving forward was def rr_ahead(speed): ... now rr.move(speed)
#rear right motor moving back    was def rr_back(speed): ...  now rr.move(-speed)

def stop_car():
    rl.brake()
    rr.brake()
    fl.brake()
    fr.brake()
    #brakes all 4 wheels

def coast_car():
    rl.move(0)
    rr.move(0)
    fl.move(0)
    fr.move(0)
    #coast all 4 wheels
      
def go_ahead(speed):
    rl.move(speed)
    rr.move(speed)
    fl.move(speed)
    fr.move(speed)

def change_speed(wheel_speeds):
    rl.move(wheel_speeds[2])
    rr.move(wheel_speeds[3])
    fl.move(wheel_speeds[0])
    fr.move(wheel_speeds[1])
    
def go_back(speed):
    rr.move(-speed)
    rl.move(-speed)
    fr.move(-speed)
    fl.move(-speed)

#making right turn on spot (tank turn)
def turn_right(speed):
    rl.move(speed)
    rr.move(-speed)
    fl.move(speed)
    fr.move(-speed)
      
#make left turn on spot (tank turn)
def turn_left(speed):
    rr.move(speed)
    rl.move(-speed)
    fr.move(speed)
    fl.move(-speed)

# parallel left shift (crab left)
def shift_left(speed):
    fr.move(speed)
    rr.move(-speed)
    rl.move(speed)
    fl.move(-speed)

# parallel right shift (crab right)
def shift_right(speed):
    fr.move(-speed)
    rr.move(speed)
    rl.move(-speed)
    fl.move(speed)

#diagonal forward and right @45
def upper_right(speed):
    rr.move(speed)
    fl.move(speed)
    
#diagonal back and left @45
def lower_left(speed):
    rr.move(-speed)
    fl.move(-speed)

#diagonal forward and left @45    
def upper_left(speed):
    fr.move(speed)
    rl.move(speed)

#diagonal back and rightt @45
def lower_right(speed):
    fr.move(-speed)
    rl.move(-speed)

'''
pwmOEn=0 #enable outputs of PCA9685
print ("By wheel")
time.sleep(2)
print ("Front Left ahead @ full speed")
fl.move(100)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Front right ahead @ full speed")
fr.move(100)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Rear Left ahead @ full speed")
rl.move(100)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Rear right ahead @ full speed")
rr.move(100)
time.sleep(2)
stop_car()
time.sleep(3)

print("Speed check, in reverse, by axle")
print ("Front axle both reverse, left at 75%, right at 40%  speed - *inital setup will go full speed")
fl.move(-75)
fr.move(-40)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Rear axle both reverse, left at 75%, right at 40%  speed")
rl.move(-75)
rr.move(-40)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Left Front 60%*, left rear at 40%, both forward")
fl.move(60)
rl.move(40)
time.sleep(2)
stop_car()
time.sleep(3)

print ("Right Front 40%*, right rear at 60%, both forward")
fr.move(40)
rr.move(60)
time.sleep(2)
stop_car()
time.sleep(3)

print("Coast All")
rl.move(0)
rr.move(0)
fr.move(0)
rr.move(0)
time.sleep(3)

print("Done")
pwmOEn=1 #disable outputs of PCA9685
GPIO.cleanup()    '''