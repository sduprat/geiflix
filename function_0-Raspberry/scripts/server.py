# coding: utf-8
from threading import Thread
import time
import can
import os
import struct
import socket

HOST = '192.168.1.2'     # IP address on LAN
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

class EthReceiver(Thread):
    def __init__(self,connSock, canBus):
        Thread.__init__(self)
        self.connSock = connSock
        self.canBus  = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.distance = 0
        self.angle = 0
        self.speed_cmd = 0
        self.movement = 0
        self.turn = 0
        self.enable_steering = 0

    def run(self):
        self.distance = 0
        self.angle = 0
        self.enable_speed = 0
        self.speed_cmd = 0
        # self.movement = 0
        # self.turn = 0
        # self.enable_steering = 0
        
        while True :
            
            # Receive data from Discovery
            fromDiscov = self.canBus.recv()
            
            # Receive data from Jetson (eth)
            fromJetson = self.connSock.recv(1024)
            
            if not fromJetson: 
                print('No data from Jetson')
                break

            # Split data between distance (1st 8 B) & angle (following 8 B)
            rawDist = fromJetson[0:8]
            rawAngle = fromJetson[8:16]
            print("Raw distance : ", rawDist, " ; Raw angle : ", rawAngle)
            if rawDist: self.distance = int(rawDist)
            if rawAngle: self.angle = int(rawAngle)
            print("Distance : ", self.distance, " ; Angle : ", self.angle)
            
            # Detect obstacle from US sensors
            if (fromDiscov.arbitration_id == US1):
                # Av Gche
                usAvG = int.from_bytes(fromDiscov.data[0:2], byteorder='big')
                # Av Dte
                usAvD = int.from_bytes(fromDiscov.data[2:4], byteorder='big')
                # Arr Centre
                usArrC = int.from_bytes(fromDiscov.data[4:6], byteorder='big')
            elif (fromDiscov.arbitration_id == US2):
                # Arr Gche
                usArrG = int.from_bytes(fromDiscov.data[0:2], byteorder='big')
                # Arr Dte
                usArrD = int.from_bytes(fromDiscov.data[2:4], byteorder='big')
                # Av Centre
                usAvC = int.from_bytes(fromDiscov.data[4:6], byteorder='big')
            
            obstacleDetected = ((self.speed_cmd > SPEED_STOP) and \
                                        ((usAvD < 50) or (usAvG < 50) or (usAvC < 50))) or \
                                ((self.speed_cmd < SPEED_STOP) and \
                                        ((usArrD < 50) or (usArrG < 50) or (usArrC < 50)))
                        
            # Update speed cmd according to the distance
            if obstacleDetected:
                self.enable_speed = 0
            else:
                if (self.distance > 2100):
                    self.enable_speed = 1
                    self.speed_cmd = 60
                elif (self.distance < 1900):
                    self.enable_speed = 1
                    self.speed_cmd = 40
                else:
                    self.enable_speed = 0
                        
            if (self.enable_speed):
                self.speed_cmd |= (1 << 7)
            else:
                self.speed_cmd &= ~(1 << 7)

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

            toNucleo = can.Message(arbitration_id=MCM,data=[self.speed_cmd, self.speed_cmd, 0,0,0,0,0,0],extended_id=False)
            self.canBus.send(toNucleo)

        print("Connexion perdue")

        stopNucleo = can.Message(arbitration_id=MCM,data=[0,0,0,0,0,0,0,0],extended_id=False)
        self.canBus.send(stopNucleo)

        self.connSock.close()

if __name__ == "__main__":

    print('Bring up CAN0....')
    os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    time.sleep(0.1)

    try:
        canBus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        print('Cannot find PiCAN board.')
        exit()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((HOST, PORT))
    sock.listen(1)
    (connSock, addr) = sock.accept()
    print('Connected by ', addr)


    recvThread = EthReceiver(connSock, canBus)
    recvThread.start()
    recvThread.join()
