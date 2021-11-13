# IMPORTS
import socket


#################
#               #
#   VARIABLES   #
#               #
#################

# UDP
UDP_IP = "192.168.1.102"
UDP_PORT = 6699
bufferSize = 1248
tab_distance = [[250 for azimuth in range(361)] * channel for channel in range(17)]
object_tall_already_detected = 0
object_short_already_detected = 0

#####################
#                   #
#   MAIN PROGRAM    #
#                   #
#####################

# UDP socket bind
sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))

# Data fetching on socket
while True:
    data_byte, addr = sock.recvfrom(bufferSize)
    data_hex = data_byte.hex()

    # Data Blocks analysing
    for data_block in range(1, 13): # In one Packet, there are 1->12 data blocks to analyse
                                    # And in one data block, there is 1 azimuth and 16 distance/reflectivity data TIMES 2 FOR REDUNDANCY
        # Azimuth
        offset_azimuth = (44+100*(data_block-1))*2 # Start at 44, then 144, 244, 344...
        azimuth = data_hex[offset_azimuth]+data_hex[offset_azimuth+1]+data_hex[offset_azimuth+2]+data_hex[offset_azimuth+3]
        azimuth = int(azimuth, 16)//100

        for channel in range(1, 17): # We can go to 33 but not yet  
            # Distance
            offset_distance = offset_azimuth + 4 + 6*channel
            distance_hex = data_hex[offset_distance]+data_hex[offset_distance+1]+data_hex[offset_distance+2]+data_hex[offset_distance+3]
            distance = float(int(distance_hex, 16)/100)
            # Reflectivity
            offset_reflectivity = offset_distance + 4
            reflectivity_hex = data_hex[offset_reflectivity]+data_hex[offset_reflectivity+1]
            reflectivity = int(reflectivity_hex, 16)
            # Check if distance is OK
            if distance < 250.0:
                tab_distance[channel][azimuth] = distance
            
    # If an object has been detected at channel 10 (13° up)
    if tab_distance[10][0] < 2.0 and object_tall_already_detected == 0:
        object_tall_already_detected = 1
        print("'Tall' object detected !")
    elif tab_distance[10][0] > 2.0 and object_tall_already_detected == 1:
        object_tall_already_detected = 0
        print("'Tall' object gone !")
    # If an object has been detected at channel 2 (13° down) and no tall object detected
    if tab_distance[2][0] < 2.0 and object_short_already_detected == 0 and object_tall_already_detected == 0:
        object_short_already_detected = 1
        print("'Short' object detected !")
    elif tab_distance[2][0] > 2.0 and object_short_already_detected == 1:
        object_short_already_detected = 0
        print("'Short' object gone !")