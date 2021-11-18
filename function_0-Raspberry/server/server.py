# coding: utf-8
from threading import Thread
import time
import can
import os
import struct
import socket

HOST = '192.168.1.10'    # Jetson IP address
PORT = 6666              # Arbitrary non-privileged port

MCM = 0x010
MS = 0x100
US1 = 0x000
US2 = 0x001
OM1 = 0x101
OM2 = 0x102

MAX_SPEED_FW = 75
MAX_SPEED_BW = 25
SPEED_STOP = 50

'''
class EthSender(Thread):

    def __init__(self,conn, bus):
        Thread.__init__(self)
        self.conn = conn
        self.bus = bus

    def run(self):
        while True :
            msg = self.bus.recv()

            #print(msg.arbitration_id, msg.data)
            st = ""

            if msg.arbitration_id == US1:
                # ultrason avant gauche
                distance = int.from_bytes(msg.data[0:2], byteorder='big')
                message = "UFL:" + str(distance) + ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # ultrason avant droit
                distance = int.from_bytes(msg.data[2:4], byteorder='big')
                message = "UFR:" + str(distance)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # ultrason arriere centre
                distance = int.from_bytes(msg.data[4:6], byteorder='big')
                message = "URC:" + str(distance)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
            elif msg.arbitration_id == US2:
                # ultrason arriere gauche
                distance = int.from_bytes(msg.data[0:2], byteorder='big')
                message = "URL:" + str(distance)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # ultrason arriere droit
                distance = int.from_bytes(msg.data[2:4], byteorder='big')
                message = "URR:" + str(distance)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # ultrason avant centre
                distance = int.from_bytes(msg.data[4:6], byteorder='big')
                message = "UFC:" + str(distance)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
            elif msg.arbitration_id == MS:
                # position volant
                angle = int.from_bytes(msg.data[0:2], byteorder='big')
                message = "POS:" + str(angle)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # Niveau de la batterie
                bat = int.from_bytes(msg.data[2:4], byteorder='big')
                message = "BAT:" + str(bat)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # vitesse roue gauche
                speed_left = int.from_bytes(msg.data[4:6], byteorder='big')
                message = "SWL:" + str(speed_left)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
                # vitesse roue droite
                # header : SWR payload : entier, *0.01rpm
                speed_right= int.from_bytes(msg.data[6:8], byteorder='big')
                message = "SWR:" + str(speed_right)+ ";"
                size = self.conn.send(message.encode())
                if size == 0: break
            elif msg.arbitration_id == OM1:
                # Yaw
                yaw = struct.unpack('>f',msg.data[0:4])
                message = "YAW:" + str(yaw[0])+ ";"
                #st += message
                size = self.conn.send(message.encode())
                if size == 0: break
                # Pitch
                pitch = struct.unpack('>f',msg.data[4:8])
                message = "PIT:" + str(pitch[0])+ ";"
                #st += message
                size = self.conn.send(message.encode())
                if size == 0: break
            elif msg.arbitration_id == OM2:
                # Roll
                roll = struct.unpack('>f',msg.data[0:4])
                message = "ROL:" + str(roll[0])+ ";"
                #st += message
                size = self.conn.send(message.encode())
                if size == 0: break

            #if (st!=""):print(st)
'''

class EthReceiver(Thread):
    def __init__(self,conn, bus):
        Thread.__init__(self)
        self.conn = conn
        self.bus  = can.interface.Bus(channel='can0', bustype='socketcan_native')

        self.distance = 0
        self.angle = 0
        self.speed_cmd = 0
        self.movement = 0
        self.turn = 0
        self.enable_steering = 0
        self.enable = 0

    def run(self):
        self.distance = 0
        self.angle = 0
        self.speed_cmd = 0
        # self.movement = 0
        # self.turn = 0
        # self.enable_steering = 0
        self.enable_speed = 0

        while True :
            # Receive data from Jetson (eth)
            data = conn.recv(1024)

            if not data: break

            # Split data between header & payload
            BDist = data[0:8]
            BAngle = data[8:16]
            self.distance = int(BDist)
            self.angle = int(BAngle)
            print("Distance : ", self.distance, " ; Angle : ", self.angle)
            
            # Update speed cmd according to the distance
            if (self.distance > 2100):
                self.enable_speed = 1
                self.speed_cmd = 60
            elif (self.distance < 1900):
                self.enable_speed = 1
                self.speed_cmd = 40
            else:
                self.enable_speed = 0
            # elif (header == b'ANG'):  # Angle

            # Receive data from Discovery
            rcvdMsg = self.bus.recv()

            # Detect obstacle from US sensors
            # if (msg.arbitration_id == US1):
            #     # Av Gche
            #     obtacleDetected = int.from_bytes(msg.data[0:2], byteorder='big') < 50
            # elif (msg.arbitration_id == US1):
            #     # Av Dte
            #     obtacleDetected = int.from_bytes(msg.data[2:4], byteorder='big') < 50
            if (msg.arbitration_id == US1):
                # Arr Centre
                obtacleDetected = (int.from_bytes(msg.data[4:6], byteorder='big') < 50) and (self.speed_cmd < SPEED_STOP)
            # elif (msg.arbitration_id == US2):
            #     # Arr Gche
            #     obtacleDetected = int.from_bytes(msg.data[0:2], byteorder='big') < 50
            # elif (msg.arbitration_id == US2):
            #     # Arr Dte
            #     obtacleDetected = int.from_bytes(msg.data[2:4], byteorder='big') < 50
            elif (msg.arbitration_id == US2):
                # Av Centre
                obtacleDetected = (int.from_bytes(msg.data[4:6], byteorder='big') < 50) and (self.speed_cmd > SPEED_STOP)

            if (self.enable_speed and not(obstacleDetected)):
                self.speed_cmd |= (1 << 7)
            else:
                self.speed_cmd &= ~(1 << 7)

            print(self.speed_cmd)
            # print(self.movement)
            # print(self.enable)
            # print(self.turn)
            # print(self.enable_steering)

            '''
            if self.enable_speed:
                cmd_mv = (50 + self.movement*self.speed_cmd) | 0x80
            else:
                cmd_mv = (50 + self.movement*self.speed_cmd) & ~0x80

            if self.enable_steering:
                cmd_turn = 50 + self.turn*30 | 0x80
            else:
                cmd_turn = 50 + self.turn*30 & 0x80

            print("mv:",cmd_mv,"turn:",cmd_turn)
            '''

            sendMsg = can.Message(arbitration_id=MCM,data=[self.speed_cmd, self.speed_cmd, 0,0,0,0,0,0],extended_id=False)

            #msg = can.Message(arbitration_id=0x010,data=[0xBC,0xBC,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
            #msg = can.Message(arbitration_id=MCM,data=[0xBC,0xBC,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
            print(sendMsg)
            self.bus.send(sendMsg)

        conn.close()

if __name__ == "__main__":

    print('Bring up CAN0....')
    os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    time.sleep(0.1)

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        print('Cannot find PiCAN board.')
        exit()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    (conn, addr) = s.accept()
    print('Connected by ', addr)


    recvThread = EthReceiver(conn, bus)
    recvThread.start()
    # sendThread = EthSender(conn, bus)
    # sendThread.start()

    recvThread.join()
    # sendThread.join()
