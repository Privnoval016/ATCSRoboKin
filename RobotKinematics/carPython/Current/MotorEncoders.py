import time #used to set delay time to control moving distance

#set up PC9685 osoyoo/AdaFruit
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685

#set up Raspberry Pi GPIO
import RPi.GPIO as GPIO #control through GPIO pins

import numpy as np

# adafruit forces GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BCM)
#I2C for PCS9685 and Gyro
# create i2c bus interface to access PCA9685, for example
i2c = busio.I2C(SCL, SDA)    #busio.I2C(board.SCL, board.SDA) create i2c bus
pca = PCA9685(i2c)           #adafruit_pca9685.PCA9685(i2c)   instance PCA9685 on bus
pca.frequency = 1000 #set pwm clock in Hz (debug 60 was 1000)
# usage: pwm_channel = pca.channels[0] instance example
#        pwm_channel.duty_cycle = speed (0 .. 100)  speed example


#motors, Swap wires for In1/In2, IN3/In4 for type B, not change here
# front controller, PCA channel
ENAFR = 0
IN1FR = 1
IN2FR = 2

IN3FL = 5
IN4FL = 6
ENBFL = 4
# rear controller, PCA channel
ENARR = 8
IN1RR = 9
IN2RR = 10

IN3RL = 13
IN4RL = 14
ENBRL = 12

#Encoders, GPIO.board pin, Swap wires for V+/Gnd for type B
S1FR = 17 #pin 11
S2FR = 27 #pin 13 

S1FL = 22 #pin 15
S2FL = 10 #pin 19

S1RR = 9  #pin 21
S2RR = 11 #pin 23

S1RL = 5  #pin 29
S2RL = 6  #pin 31
perRev = 425 # estimate A type motors

#PCA9685
PWMOEN = 4 #pin 7 #PCA9685 OEn pin
pwmOEn = GPIO.setup(PWMOEN, GPIO.OUT)  # enable PCA outputs

#push button
pushButton = 26 #pin 37, GPIO 26
GPIO.setup(pushButton, GPIO.IN)  
oldPushb = 0

# for IN1, IN2, define 1  and 0 settings
high = 0xFFFF #1 was True
low  = 0      #0 was False
# end of globals

#utility
def readPush():
  global oldPushb
  pushb = GPIO.input(pushButton)
  if pushb != oldPushb :
    oldPushb = pushb
    return True, pushb
  else :
    return False, pushb
    
#equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop): 
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

#for 0 to 100, % speed as integer, to use for PWM 
#full range 0xFFFF, but PCS9685 ignores last Hex digit as only 12 bit resolution)
def getPWMPer(value): 
  return int(valmap(value, 0, 100, 0, 0xFFFF))
#end of utility


#encoder class 
class Encoder:
  def __init__(self, name, S1, S2, side):
    self.name = name #for debug
    self.s1  = S1 #pin
    GPIO.setup(S1, GPIO.IN) #instance
    self.s2  = S2 #pin
    GPIO.setup(S2, GPIO.IN)  
    self.aState = 0 #value (aState)
    self.bState = 0
    self.aLastState = 0 # remember last value (aLastState)
    self.counter = 0
    self.lastCounter = 0
    self.speed = 0
    self.time = time.perf_counter_ns()
    self.lastTime = self.time
    self.side = side
    GPIO.add_event_detect(self.s1, GPIO.BOTH,callback = self.callback_encoder)
    	
  def read(self):
    self.aState  = GPIO.input(self.s1)
    self.bState  = GPIO.input(self.s2)
    self.time = time.perf_counter_ns()
    return self.aState, self.bState
    
  def read_turn(self):
    return self.aturn, self.bturn  # for diagnostic (whole method)
    
  def name(self):
    return self.name # for diagnostic (whole method)
    
  def readEncoder(self):
    #Reads the "current" state of the encoders
    aState, bState = self.read() 
    #If the previous and the current state are different,  a Pulse has occured
    if aState != self.aLastState:
      self.counter += 1
#      self.aturn +=1; #for diagnostic # for A only
      #If the outputB state is different to the outputA state, rotating clockwise
#      if bState!= aState :
#        self.counter += 1
#      else : #rotating counter clockwise
#        self.counter -= 1       
    self.aLastState = aState #remember last state of a

  def readEncoderTest(self):
    self.readEncoder()
    print(self.name +" position: " + str(self.counter)  ) #for diagnostic
       
  #set up call back functions, ignore 2nd parameter 
  def callback_encoder(self,channel):
    self.readEncoder()
    
  def readSpeed(self, wheel_rad):
    #correct for side of car left goes (- otherwise). Side not needed with single encoder as always positive
    if self.time != 0 and self.time != self.lastTime: #store speed in clicks/nS
  #    self.speed = self.side * (self.counter - self.lastCounter)/(self.time - self.lastTime)
      self.speed =  (self.counter - self.lastCounter)/(self.time - self.lastTime)
    else:
      self.speed = 0

    # lastTime and lastCounter were set at last call to this function
    self.lastTime = self.time #time was set at each/last call to .read
    self.lastCounter = self.counter #counter was set at each/last call to .readEncoder

    #print(self.name +" position: " + str(self.counter) + " @: " + str(self.time) + \
    #   " speed rev/sec: " + str(self.speed*1E9/perRev) ) #for diagnostic
    # print counts revs /second

    return self.speed * 1E9 / perRev * 2 * np.pi * wheel_rad
    
  def resetSpeed(self):
    self.speed = 0  
    self.counter = 0
    self.lastCounter = 0
    #may need initialize since stop so 1st speed valid
    self.time = 0
    self.lastTime = 0   
#end of Encoder class 

class Motor(Encoder):
  #inherits from Encoder
  def __init__(self, name, enCh,in1Ch,in2Ch, S1, S2, side):
    super().__init__( name, S1, S2, side)
    self.name = name #for debug
    self.en  = pca.channels[enCh]  #EN  wheel 'speed', actually sets power
    self.in1 = pca.channels[in1Ch] #IN1, IN3 wheel direction control 1
    self.in2 = pca.channels[in2Ch] #IN2, IN4 wheel direction control 2
    self.pid = 0
   # self.s = Encoder(name, S1, S2, side)

  def move(self,power):
  	self.in1.duty_cycle = high if power > 0 else low
  	self.in2.duty_cycle = low  if power > 0 else high
  	self.en.duty_cycle  = getPWMPer(power) if power > 0 else getPWMPer(-power)
  	#positive power forward, negative power reverse/back; 0 = coast, 100% = 4095
  	
  def brake(self):
  	self.in1.duty_cycle = low
  	self.in2.duty_cycle = low
  	#electric braking effect, should stop movement

  def setPID(self, x):
      self.pid = x
#end of Motor class

class FourWheel:

  def __init__(self):
    #Set up Motor instances with connections, ch 0 is left end, 
    self.rl = Motor("rl", ENBRL, IN3RL, IN4RL, S1RL, S2RL,-1) #Rear-left wheel
    self.rr = Motor("rr", ENARR, IN1RR, IN2RR, S1RR, S2RR, 1) #Rear-right wheel
    self.fl = Motor("fl", ENBFL, IN3FL, IN4FL, S1FL, S2FL, -1) #Front-left wheel
    self.fr = Motor("fr", ENAFR, IN1FR, IN2FR, S1FR, S2FR, 1) #Front-right wheel
    
  def stop_car(self):    #brakes all 4 wheels
    self.rl.brake()
    self.rr.brake()
    self.fl.brake()
    self.fr.brake()
    time.sleep(1) #allow time to halt, then reset all speeds to 0
    self.fl.resetSpeed()
    self.rr.resetSpeed()
    self.rl.resetSpeed()
    self.fr.resetSpeed()
    # note may need to resetSpeed at start of a new action, *** to check if needed
    #      coast does not reset speed, deliberately, as car may still move.

  def change_speed(self, wheel_speeds):
      self.rl.move(wheel_speeds[2])
      self.rr.move(wheel_speeds[3])
      self.fl.move(wheel_speeds[0])
      self.fr.move(wheel_speeds[1])

  def coastAll(self, forSecs):
      print("Coast forSecs " + str(forSecs) + " secs.")
      self.rl.move(0)
      self.rr.move(0)
      self.fr.move(0)
      self.rr.move(0)
      time.sleep(forSecs)
      self.stop_car()  # will set speed to 0
#end of FourWheel class

#test operations
def test_speed():
  car.fl.readSpeed() 
  car.fr.readSpeed() 
  car.rl.readSpeed() 
  car.rr.readSpeed()

def test_Encoders():
  car.fl.readEncoderTest() 
  car.fr.readEncoderTest() 
  car.rl.readEncoderTest() 
  car.rr.readEncoderTest() 
  
def test_move():
	pwmOEn=0 #enable outputs of PCA9685
	print ("By wheel")
	time.sleep(2)
	
	print ("Front Left ahead @ fullpower")
	car.fl.move(100)
	time.sleep(1) #allow time to get to full speed
	test_speed()
	time.sleep(1) #complete time for action
	car.stop_car()    #will set speed to 0
	time.sleep(3) # for operators benefit

	print ("Front right ahead @ full power")
	car.fr.move(100)
	time.sleep(1)
	test_speed()
	time.sleep(1)
	car.stop_car()
	time.sleep(3)

	print ("Rear Left ahead @ full power")
	car.rl.move(100)
	time.sleep(1)
	test_speed()
	time.sleep(1)
	car.stop_car()
	time.sleep(3)

	print ("Rear right ahead @ full power")
	car.rr.move(100)
	time.sleep(1)
	test_speed()
	time.sleep(1)
	car.stop_car()
	time.sleep(3)

	print("Run backward all, full power for 1 sec, then coast all for 3 sec, then stop")
	car.fl.move(-100)
	car.fr.move(-100)
	time.sleep(1)
	car.rl.move(-100)
	car.rr.move(-100)
	time.sleep(1)
	test_speed()
	car.rl.move(0)
	car.rr.move(0)
	car.fl.move(0)
	car.fr.move(0)
	time.sleep(3)
	test_speed()
	car.stop_car() # will reset speed
	test_speed()
	print("Stopped and speed reset")

def test_readPush():
    changed, state = readPush()
    if changed :
      print ("button changed to " + str(state))	
# end of tests of operations

#main control
def destroy():
    pwmOEn=1 #disable outputs of PCA9685
    GPIO.cleanup()
    
def main(): 
    print("starting main")
    while True:
   #   test_readPush()
      test_move()
   #   test_Encoders()
  
if __name__ == '__main__':
    try:
      main()
    except KeyboardInterrupt:
      car.stop_car() #stop movement
      destroy()  #clean up GPIO
      print("\nDone")    