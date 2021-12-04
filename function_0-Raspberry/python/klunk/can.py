import can
import klunk as k
import klunk.motors

def can_message(can_id, can_data):
    return can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)

def motors_message(speed, steer):
    if speed not in k.motors.SPEEDS or steer not in k.motors.STEERS:
        raise ValueError("Invalid speed or steer value")

    return can_message(k.motors.CAN_ID, [speed, steer])

def is_message_from_ids(message, ids):
    if isinstance(ids, list):
        return message.arbitration_id in ids
    else:
        return message.arbitration_id == ids

