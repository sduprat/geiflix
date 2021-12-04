#!/usr/bin/python3

import can

import klunk as k
import klunk.can
import klunk.motors
import klunk.ultrasound

bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
ultrasound = k.ultrasound.Ultrasound()

# Main loop
try:
    while True:
        received_message = bus.recv()

        if k.can.is_message_from_ids(received_message, k.ultrasound.CAN_IDS):
            ultrasound.update(received_message)

        print(ultrasound.values)

        if ultrasound.front_center_obstacle() \
            or (ultrasound.front_left_obstacle() and ultrasound.front_right_obstacle()):
            speed = k.motors.SPEED_STOP
            steer = k.motors.STEER_STRAIGHT
            print("ordre arret")
        elif ultrasound.front_right_obstacle():
            speed = k.motors.SPEED_SLOW
            steer = k.motors.STEER_LEFT_FAR
            print("ordre gauche")
        elif ultrasound.front_left_obstacle():
            speed = k.motors.SPEED_SLOW
            steer = k.motors.STEER_RIGHT_FAR
            print("ordre droit")
        else:
            speed = k.motors.SPEED_SLOW
            steer = k.motors.STEER_STRAIGHT
            print("ordre d'avancer")

        bus.send(k.can.motors_message(speed, steer))

except KeyboardInterrupt:
    bus.send(k.can.motors_message(k.motors.SPEED_STOP, k.motors.STEER_STRAIGHT))

