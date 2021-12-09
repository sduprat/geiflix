#!/usr/bin/python3

import can
import time

import klunk as k
import klunk.can
import klunk.car
import klunk.motors

bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
car = k.car.Car(bus)

last_order = None

# Main loop
try:
    while True:
        with open('/var/www/Klunk/action.txt') as file:
            action = file.read().strip()
                
        if action == 'forward':
            car.set_speed(k.motors.SPEED_SLOW)
        elif action == 'left':
            if last_order != action:
                car.lefter()
        elif action == 'right':
            if last_order != action:
                car.righter()
        elif action == 'reverse':
            car.set_speed(k.motors.SPEED_REVERSE)
        else:
            car.brake()

        print(f"action : {repr(action)}, speed = {hex(car.speed)}, steer = {hex(car.steer)}")
        last_order = action
        time.sleep(0.01)
except KeyboardInterrupt:
    bus.send(k.can.motors_message(k.motors.SPEED_STOP, k.motors.STEER_STRAIGHT))

