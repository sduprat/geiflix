#!/usr/bin/python3
#
# simple_tx_test.py
#
# This python3 sent CAN messages out, with byte 7 increamenting each time.
# For use with PiCAN boards on the Raspberry Pi
# http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-2-p-1475.html
#
# Make sure Python-CAN is installed first http://skpang.co.uk/blog/archives/1220
#
# 01-02-16 SK Pang
#
#
#


import RPi.GPIO as GPIO
import can
import time
import os


led = 22
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(led,GPIO.OUT)
GPIO.output(led,True)

count = 0

print('\n\rCAN Rx test')
print('Bring up CAN0....')

US1 = 0x000
US2 = 0x001

THRESHOLD_OBSTACLE = 50

distance = [[0, 0, 0],[0, 0, 0]]

# Bring up can0 interface at 400kbps
os.system("sudo /sbin/ip link set can0 up type can bitrate 400000")
time.sleep(0.1)
print('Press CTL-C to exit')

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    GPIO.output(led,False)
    exit()

# Main loop
try:
    while True:
        msg = bus.recv()

        if msg.arbitration_id == US1:
            # ultrason avant gauche
            distance[0][0] = int.from_bytes(msg.data[0:2], byteorder='big')
            # ultrason avant droit
            distance[0][2] = int.from_bytes(msg.data[2:4], byteorder='big')
            # ultrason arriere centre
            distance[1][1] = int.from_bytes(msg.data[4:6], byteorder='big')
        elif msg.arbitration_id == US2:
            # ultrason arriere gauche
            distance[1][0] = int.from_bytes(msg.data[0:2], byteorder='big')
            # ultrason arriere droit
            distance[1][2] = int.from_bytes(msg.data[2:4], byteorder='big')
            # ultrason avant centre
            distance[0][1] = int.from_bytes(msg.data[4:6], byteorder='big')

        print("Matrice de detection US (cm) \n"+"["+ str(distance[0][0])+", "+str(distance[0][1])+", "+str(distance[0][2])+"]\n"+"["+ str(distance[1][0])+", "+str(distance[1][1])+", "+str(distance[1][2])+"]\n")

        obstacle_front = 0b0000 #matrice des obstacles détectés vers l avant de la voiture

        #la boucle marche pas mais je sais pas pourquoi
        #for j in range(0,2):
            ##if distance[0][j]<THRESHOLD_OBSTACLE:
                ##obstacle_front = obstacle_front + 0b0001;
                ##print("Obstacle inside : "+str(obstacle_front)+"\n")
            ##obstacle_front = obstacle_front<<1;
        if distance[0][0]<THRESHOLD_OBSTACLE:
            obstacle_front = obstacle_front + 0b0001;
            print("Obstacle gauche ? : "+str(obstacle_front)+"\n")
        
        obstacle_front = obstacle_front<<1;
        
        if distance[0][1]<THRESHOLD_OBSTACLE:
            obstacle_front = obstacle_front + 0b0001;
            print("Obstacle milieu ? : "+str(obstacle_front)+"\n")
        
        obstacle_front = obstacle_front<<1;
        
        if distance[0][2]<THRESHOLD_OBSTACLE:
            obstacle_front = obstacle_front + 0b0001;
            print("Obstacle milieu ? : "+str(obstacle_front)+"\n")
        
        obstacle_front = obstacle_front<<1;
        
        print("Obstacles : "+str(obstacle_front)+"\n")
        
        if obstacle_front == 0b0000:
            msg = can.Message(arbitration_id=0x020,data=[0x37,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b0010:
            msg = can.Message(arbitration_id=0x020,data=[0x37,0x0A,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b1000:
            msg = can.Message(arbitration_id=0x020,data=[0x37,0x5A,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b0100:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b1100:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b0110:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b1110:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        elif obstacle_front == 0b1010:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
        else:
            msg = can.Message(arbitration_id=0x020,data=[0x32,0x32,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)

        bus.send(msg)
        
        #Pour verifier que la Raspi est ON, on allume une led
        #GPIO.output(led,True)
        #count +=1
        #time.sleep(0.1)
        #GPIO.output(led,False)
        #time.sleep(0.1)
        #print(count)



except KeyboardInterrupt:
    #Catch keyboard interrupt
    GPIO.output(led,False)
    os.system("sudo /sbin/ip link set can0 down")
    print('\n\rKeyboard interrupt: CAN down')
