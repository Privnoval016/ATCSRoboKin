#!/usr/bin/env python
"""
    Simple simulation of a water boiler which can heat up water
    and where the heat dissipates over time, to explore PID
    author: M baynes modified
    last change: Feb 27, 2023
"""

import time
import matplotlib.pyplot as plt
from simple_pid import PID

start_temp = 20
goal_temp = 100
temp_loss = 10  # was 0.01
Kp = 200  # was 5
Ki = 0.01  # was 0.01
Kd = 0.01  # was 0.01
boiler_power = 50  # was 100
time_run = 10
turn_on = 1
pid_auto_mode = True


class WaterBoiler:
    def __init__(self):
        self.water_temp = start_temp

    def update(self, boiler_power, dt):
        if boiler_power > 0:
            # Boiler can only produce heat, not cold
            self.water_temp += 1 * boiler_power * dt

        # Some heat dissipation
        self.water_temp -= temp_loss * dt  # was 0.02
        return self.water_temp


if __name__ == '__main__':
    boiler = WaterBoiler()
    water_temp = boiler.water_temp
    #         P     I   D    goal
    pid = PID(Kp, Ki, Kd, setpoint=water_temp)
    #    pid = PID(5, 0.01, 0.1, setpoint=water_temp)
    pid.output_limits = (0, boiler_power)
    start_time = time.time()
    last_time = start_time

    # Keep track of values for plotting
    powers, setpoint, y, x = [], [], [], []
    errors, pa, ia, da = [], [], [], []
    state = 0  # off
    while time.time() - start_time < time_run:
        current_time = time.time()
        dt = current_time - last_time

        power = pid(water_temp)
        water_temp = boiler.update(power, dt)

        p, i, d = pid.components
        pa += [p]
        ia += [i]
        da += [d]
        errors += [pid.setpoint - water_temp]

        x += [current_time - start_time]
        y += [water_temp]
        setpoint += [pid.setpoint]
        powers += [power]

        if not state:
            if current_time - start_time > turn_on:
                pid.setpoint = goal_temp
                state = 1

        last_time = current_time

    # run is complete, plot the results
    plt.plot(x, y, label='measured')
    plt.plot(x, setpoint, label='target')
    plt.plot(x, powers, label='power')
    plt.xlabel('time')
    plt.ylabel('temperature')
    plt.legend()

    plt.figure()
    plt.plot(x, errors, label='error')
    plt.plot(x, pa, label='P')
    plt.xlabel('time')
    plt.ylabel('')
    plt.legend()

    plt.figure()
    plt.plot(x, ia, label='i')
    plt.plot(x, da, label='d')
    plt.xlabel('time')
    plt.ylabel('')
    plt.legend()
    plt.show()
