# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time

import numpy as np

#set up PC9685 osoyoo/AdaFruit
#from board import SCL,SDA
SCL = 3
SDA = 2

import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

#equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop): 
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x41)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)
armJoint = [0,0,0,0,0]
for i in range(0, len(armJoint)):
  armJoint[i] = servo.Servo(pca.channels[i], min_pulse=400, max_pulse=2400)

offsets = [90, 110, 90, 90, 90]
angles = [
    [0, 0, 0, 0, 0],
    [ 2.30520227e-06, -1.76967835e+01,  1.59237607e+01, -90, 0],
    [ 2.30520227e-06, -1.76967835e+01,  1.59237607e+01, 90, 0],

    [ 5.08888749e-14, -2.57664451e+01,  2.20397022e+01,  0.00000000e+00, 0],
    [  0.,         -24.81090086,  33.87437146,   0.        , 0],
    [ 5.08888749e-14, -2.03582370e+01,  3.05550464e+01,  5.08888749e-14, 0]
]

# We sleep in the loops to give the servo time to move into position.
#j=0
for j in range(0, len(angles)):
    for i in range(0, len(armJoint)):
        armJoint[i].angle = angles[j][i] + offsets[i]
    time.sleep(0.5)
 
pca.deinit()
