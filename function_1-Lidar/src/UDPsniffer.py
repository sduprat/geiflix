# IMPORTS
import socket
import re


#################
#               #
#   VARIABLES   #
#               #
#################

# UDP
UDP_IP = "192.168.1.102"
UDP_PORT = 6699
bufferSize = 1248

# MSOP Packet
MSOP_Header = '55aa050a5aa550a0'
lenght_MSOP_Header = 16

offset_Time_Code = lenght_MSOP_Header + 24
lenght_Time_Code = 20

offset_LIDAR_Type = offset_Time_Code + lenght_Time_Code
lenght_LIDAR_Type = 2

Start_Identifier = 'ffee'
End_Identifier = '00ff'

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
data_byte, addr = sock.recvfrom(bufferSize)
data_hex = data_byte.hex()
print("Received message: %s" % data_hex)
print("\n\n\n")

# Data analysing
reMSOP_Header = re.compile(MSOP_Header) # Compile MSOP header (string) into a regular expression to search
for match_obj in reMSOP_Header.finditer(data_hex):
    offset_start = match_obj.start()
    offset_end = match_obj.end()
    print("MSOP Header found from byte %d to %d " % (offset_start/2, offset_end/2))

print("MSOP Header : ", end = '')
for i in range(lenght_MSOP_Header) :
    print("%s" % data_hex[i+offset_start], end = '')

print("")

print("TimeCode : ", end = '')
for i in range(lenght_Time_Code) :
    print("%s" % data_hex[i+offset_start+offset_Time_Code], end = '')

print("")

print("Lidar Type : ", end = '')
for i in range(lenght_LIDAR_Type) :
    print("%s" % data_hex[i+offset_start+offset_LIDAR_Type], end = '')

print("")

reStart_Identifier = re.compile(Start_Identifier)
for match_obj in reStart_Identifier.finditer(data_hex):
    offset_start = match_obj.start()
    offset_end = match_obj.end()
    print("Start identifier found from byte %d to %d " % (offset_start/2, offset_end/2))

reEnd_Identifier = re.compile(End_Identifier)
for match_obj in reEnd_Identifier.finditer(data_hex):
    offset_start = match_obj.start()
    offset_end = match_obj.end()
    print("End identifier found from byte %d to %d " % (offset_start/2, offset_end/2))

print("")

# Data Blocks analysing
for i in range(1, 13):  # In one Packet, there are 1->12 data blocks to analyse
                        # And in one data block, there is 1 azimuth and 16 distance/reflectivity data TIMES 2 FOR REDUNDANCY
        # Azimuth
    offset_azimuth = (44+100*(i-1))*2 # Start at 44, then 144, 244, 344...
    azimuth = data_hex[offset_azimuth]+data_hex[offset_azimuth+1]+data_hex[offset_azimuth+2]+data_hex[offset_azimuth+3]
    azimuth = int(azimuth, 16)//100
    distance = [250 for i in range(33)]
    reflectivity = [255 for i in range(33)]

    for j in range(1, 17): # We can go to 33 but not yet  
            # Distance
        offset_distance = offset_azimuth + 4 + 6*j
        distance_hex = data_hex[offset_distance]+data_hex[offset_distance+1]+data_hex[offset_distance+2]+data_hex[offset_distance+3]
        distance[j] = float(int(distance_hex, 16)/100)
            # Reflectivity
        offset_reflectivity = offset_distance + 4
        reflectivity_hex = data_hex[offset_reflectivity]+data_hex[offset_reflectivity+1]
        reflectivity[j] = int(reflectivity_hex, 16)

        if distance[j] > 250: distance[j] = 0

for i in range(9, 17):
        # Print Distance
    print(f"Azimuth {azimuth}°, channel n. {i} : {distance[i]}m vs {distance[i+16]}m")
for i in range(8, 0, -1):
        # Print Distance
    print(f"Azimuth {azimuth}°, channel n. {i} : {distance[i]}m vs {distance[i+16]}m")


