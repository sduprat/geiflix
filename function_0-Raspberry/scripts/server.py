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
        usAvG,usAvD,usArrC,usArrG,usArrD,usAvC = 0,0,0,0,0,0
        
        while True :
            
            # Receive data from Discovery
            fromDiscov = self.canBus.recv()
            
            # Receive data from Jetson (eth)
            fromJetson = self.connSock.recv(1024)
            
            # Leave if no more data coming from Jetson (= eth link lost)
            if not fromJetson: 
                print('No data from Jetson')
                break

            # Split data between distance (1st 8 B) & angle (following 8 B)
            rawDist = fromJetson[0:8]
            rawAngle = fromJetson[8:16]
            if rawDist: self.distance = int(rawDist)
            if rawAngle: self.angle = int(rawAngle)
            print("d : ", self.distance, " ; a : ", self.angle)

            # Update speed cmd according to the distance
            self.enable_speed = True
            erreur = self.distance - 2000
            if (erreur<-2000):
                self.speed_cmd = 25
            elif (erreur > 2000):
                self.speed_cmd = 75
            else:
                self.speed_cmd = 0.0125*erreur + 50

            
            # Get US sensors values
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
            
            # Determine obstacle presence given direction (FW/BW) & US values
            obstacleDetected = ((self.speed_cmd > SPEED_STOP) and \
                                        ((usAvD < 50) or (usAvG < 50) or (usAvC < 50))) or \
                                ((self.speed_cmd < SPEED_STOP) and \
                                        ((usArrD < 50) or (usArrG < 50) or (usArrC < 50)))
            self.enable_speed &= not obstacleDetected
                        
            # Enable or not speed in motor command
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
            
            # Compose & send CAN message to Nucleo
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
