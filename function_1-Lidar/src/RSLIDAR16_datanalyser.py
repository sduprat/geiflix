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
# LIDAR
bufferSize = 1248
tab_distance = [[255 for azimuth in range(36001)] * channel for channel in range(17)]
new_azimuth = 0; old_azimuth = 0; missing_azimuth = 0
nb_nearby_objects = 0
object_detected = 0

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
        azimuth = int(azimuth, 16)

        for channel in range(1, 17): # We can go to 33 but not yet  
            # Distance (/100 for meters, /2 for one-way distance)
            offset_distance = offset_azimuth + 4 + 6*channel
            distance_hex = data_hex[offset_distance]+data_hex[offset_distance+1]+data_hex[offset_distance+2]+data_hex[offset_distance+3]
            distance = float(int(distance_hex, 16)/100)/2
            # Reflectivity
            offset_reflectivity = offset_distance + 4
            reflectivity_hex = data_hex[offset_reflectivity]+data_hex[offset_reflectivity+1]
            reflectivity = int(reflectivity_hex, 16)

            # Check if distance is OK, 0.2m min and 150m max
            if distance > 0.2 and distance < 150.0 :
                tab_distance[channel][azimuth] = distance

        print(f"Azimuth : {azimuth}")


        #         # Perform interpolation
        #         new_azimuth = azimuth
        #             # Adjust for rollover from 359.9° to 0°
        #         if new_azimuth < old_azimuth:
        #             new_azimuth += 3600
        #             # Calculate missing azimuth
        #         missing_azimuth = old_azimuth + (new_azimuth-old_azimuth)//2
        #             #Adjust for rollover from 359.9° to 0°
        #         if missing_azimuth > 3600:
        #             missing_azimuth -= 3600
        #             # Calculate mean distance between old azimuth and new azimuth
        #         tab_distance[channel][missing_azimuth] = (tab_distance[channel][old_azimuth]+tab_distance[channel][azimuth])/2
                    
        # old_azimuth = azimuth

        # print(f"MODIF chan. {16}, azimuth {missing_azimuth}, distance = {tab_distance[16][missing_azimuth]}")
        # print(f"For channel {16}, azimuth {azimuth}, distance = {tab_distance[16][azimuth]}")
        
    # Detection of nearby objects
    # for azimuth in range(361):
    #     if tab_distance[16][azimuth] < 0.5 :
    #         nb_nearby_objects += 1

    # if nb_nearby_objects == 0 and object_detected == 1:
    #     print("Nearby Object Gone !")
    #     object_detected = 0

    # if nb_nearby_objects > 0 and object_detected == 0:
    #     print("Nearby Object Detected !")
    #     object_detected = 1

    # nb_nearby_objects = 0