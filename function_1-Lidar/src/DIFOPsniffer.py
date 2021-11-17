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
UDP_DIFOP_PORT = 7788
bufferSize = 1248

# DIFOP Packet
DIFOP_Header = 'a5ff005a11115555'
lenght_DIFOP_Header = 16

offset_Motor_Speed = lenght_DIFOP_Header
lenght_Motor_Speed = 4

#####################
#                   #
#   MAIN PROGRAM    #
#                   #
#####################

# UDP socket bind
sock_DIFOP = socket.socket(socket.AF_INET,    # Internet
                           socket.SOCK_DGRAM) # UDP
sock_DIFOP.bind((UDP_IP, UDP_DIFOP_PORT))

# Data fetching on socket
data_byte, addr = sock_DIFOP.recvfrom(bufferSize)
data_hex = data_byte.hex()
print("DIFOP message: %s" % data_hex)
print("\n\n\n")


# Data analysing
reDIFOP_Header = re.compile(DIFOP_Header) # Compile DIFOP header (string) into a regular expression to search
for match_obj in reDIFOP_Header.finditer(data_hex):
    offset_start = match_obj.start()
    offset_end = match_obj.end()
    print("MSOP Header found from byte %d to %d " % (offset_start/2, offset_end/2))

print("MSOP Header : ", end = '')
for i in range(lenght_DIFOP_Header) :
    print("%s" % data_hex[i+offset_start], end = '')

print("")

print("Rotation Speed : ", end = '')
for i in range(lenght_Motor_Speed) :
    print("%s" % data_hex[i+offset_start+offset_Motor_Speed], end = '')

print("")