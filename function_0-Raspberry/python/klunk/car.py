import klunk as k
import klunk.can
import klunk.motors

class Car:
    def __init__(self, can_bus):
        self.speed = k.motors.SPEED_STOP
        self.steer = k.motors.STEER_STRAIGHT
        self.can_bus = can_bus
        self.send_motors_order()

    def send_motors_order(self):
        self.can_bus.send(k.can.motors_message(self.speed, self.steer))

    def set_speed(self, speed):
        self.speed = speed
        self.send_motors_order()

    def brake(self):
        self.set_speed(k.motors.SPEED_STOP)

    def faster(self):
        self.set_speed(k.motors.faster(self.speed))
        
    def slower(self):
        self.set_speed(k.motors.slower(self.speed))

    def set_steer(self, steer):
        self.steer = steer
        self.send_motors_order()

    def lefter(self):
        self.set_steer(k.motors.lefter(self.steer))

    def righter(self):
        self.set_steer(k.motors.righter(self.steer))

