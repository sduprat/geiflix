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
        with open('/var/www/Klunk/order.txt') as file:
            order = file.read().strip()
            if last_order is None:
                last_order = order
        if order != last_order:
            if order == 'reverse':
                car.set_speed(k.motors.SPEED_REVERSE)
            elif order == 'stop':
                car.set_speed(k.motors.SPEED_STOP)
            elif order == 'speed1':
                car.set_speed(k.motors.SPEED_SLOW)
            elif order == 'speed2':
                car.set_speed(k.motors.SPEED_MEDIUM)
            elif order == 'speed3':
                car.set_speed(k.motors.SPEED_FAST)

            elif order == 'left3':
                car.set_steer(k.motors.STEER_LEFT_FAR)
            elif order == 'left2':
                car.set_steer(k.motors.STEER_LEFT_MIDDLE)
            elif order == 'left1':
                car.set_steer(k.motors.STEER_LEFT_CLOSE)
            elif order == 'straight':
                car.set_steer(k.motors.STEER_STRAIGHT)
            elif order == 'right1':
                car.set_steer(k.motors.STEER_RIGHT_CLOSE)
            elif order == 'right2':
                car.set_steer(k.motors.STEER_RIGHT_MIDDLE)
            elif order == 'right3':
                car.set_steer(k.motors.STEER_RIGHT_FAR)

            else:
                car.brake()
            last_order = order

            print(f"order : {repr(order)}, speed = {hex(car.speed)}, steer = {hex(car.steer)}")
        time.sleep(0.01)
except KeyboardInterrupt:
    bus.send(k.can.motors_message(k.motors.SPEED_STOP, k.motors.STEER_STRAIGHT))

