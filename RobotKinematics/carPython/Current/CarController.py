#
__author__ = "Pranav Sukesh"
__date__ = "2025-01-25"

import numpy as np

from MotorEncoders import *
from CarPathGen import *
from simple_pid import PID
import time


Kp = 50
Ki = 0
Kd = 0

max_speed = 100


def car_update(frame):
    frame_speeds = np.empty(4)
    for wheel in range(4):
        motors[wheel].pid.setpoint = wheel_speeds[frame][wheel]
        spd = motors[wheel].readSpeed(wheel_rad)
        frame_speeds[wheel] = max(-max_speed,
                                  min(max_speed, wheel_speeds[frame][wheel] +
                                      time_per_update *
                                      motors[wheel].pid(spd)))

    if (abs(frame_speeds[0]) > max_speed or abs(frame_speeds[1]) > max_speed or
            abs(frame_speeds[2]) > max_speed or
            abs(frame_speeds[3]) > max_speed):
        return
    if (abs(frame_speeds[0]) == 0 or abs(frame_speeds[1]) == 0 or
            abs(frame_speeds[2]) == 0 or abs(frame_speeds[3]) == 0):
        return

    car.change_speed(frame_speeds)


def main():
    global time_per_update, wheel_rad
    time_per_update, plot_vel, plot_dir, wheel_rad = read_info()

    global points, wheel_speeds
    points, wheel_speeds = start()

    global car, motors

    car = FourWheel()
    motors = [car.fl, car.fr, car.rl, car.rr]

    for i in range(4):
        x = PID(Kp, Ki, Kd, setpoint=wheel_speeds[0][i])
        x.output_limits = (-2 * max_speed, 2 * max_speed)
        motors[i].setPID(x)

    for i in range(0, len(points) - 1):
        car_update(i)
        time.sleep(time_per_update)

    car.coastAll(0.2)

    GPIO.cleanup()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Simulation stopped.")
        car.coastAll(0.2)
        pwmOEn = 1
        GPIO.cleanup()
