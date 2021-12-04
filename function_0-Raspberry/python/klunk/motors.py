CAN_ID = 0x020

SPEED_REVERSE = 0x28
SPEED_STOP = 0x32
SPEED_SLOW = 0x3C
SPEED_MEDIUM = 0x41
SPEED_FAST = 0x4B

SPEEDS = [
    SPEED_REVERSE,
    SPEED_STOP,
    SPEED_SLOW,
    SPEED_MEDIUM,
    SPEED_FAST
]

STEER_LEFT_FAR = 0x0A
STEER_LEFT_MIDDLE = 0x19
STEER_LEFT_CLOSE = 0x28
STEER_STRAIGHT = 0x32
STEER_RIGHT_CLOSE = 0x3C
STEER_RIGHT_MIDDLE = 0x4B
STEER_RIGHT_FAR = 0x5A

STEERS = [
    STEER_LEFT_FAR,
    STEER_LEFT_MIDDLE,
    STEER_LEFT_CLOSE,
    STEER_STRAIGHT,
    STEER_RIGHT_CLOSE,
    STEER_RIGHT_MIDDLE,
    STEER_RIGHT_FAR
]

def slower(speed):
    if speed == SPEED_REVERSE:
        return SPEED_STOP
    else:
        return SPEEDS[max(0, SPEEDS.index(speed) - 1)]

def faster(speed):
    return SPEEDS[min(SPEEDS.index(speed) + 1, len(SPEEDS) - 1)]

def lefter(steer):
    return STEERS[max(0, STEERS.index(steer) - 1)]

def righter(steer):
    return STEERS[min(STEERS.index(steer) + 1, len(STEERS) - 1)]
