# IMPORTS
import socket


#### WARNING ####     #### WARNING ####     #### WARNING ####
#### WARNING ####     #### WARNING ####     #### WARNING ####
#### WARNING ####     #### WARNING ####     #### WARNING ####

# ONLY USE THIS CODE WITH LOCAL IP ADDRESS:PORT 127.0.0.1:5005
# TO USE THE SNIFFER, GO TO THE SRC FOLDER, OPEN A TERMINAL AND TYPE 'python3 UCWPsniffer.py'

#### WARNING ####     #### WARNING ####     #### WARNING ####
#### WARNING ####     #### WARNING ####     #### WARNING ####
#### WARNING ####     #### WARNING ####     #### WARNING ####


#################
#               #
#   VARIABLES   #
#               #
#################

# UDP
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# UCWP Packet
UCWP_Header     = b'aa00ff112222aaaa'
Rotation_Speed  = b'04B0'
LIDAR_IP        = b'C0A801C8'
DEST_PC_IP      = b'C0A80166'
MAC_ADDR        = b'001C23174ACC'
MSOP_Port1      = b'1A2B'
MSOP_Port2      = b'1A2B'
DIFOP_Port3     = b'1E6C'
DIFOP_Port4     = b'1E6C'
Other_Ports     = b'00000000'
UTC_TIME        = b'11030A092D1E006400C8'
Others          = b'0000'
Motor_Phase_Lck = b'005A'
Others_long     = b'00'
for i in range(1195):
    Others_long += b'00'
UCWP_Tail       = b'0ff0'

#####################
#                   #
#   MAIN PROGRAM    #
#                   #
#####################

message =  (UCWP_Header + 
            Rotation_Speed + 
            LIDAR_IP + DEST_PC_IP + MAC_ADDR +
            MSOP_Port1 + MSOP_Port2 +
            DIFOP_Port3 + DIFOP_Port4 +
            Other_Ports +
            UTC_TIME +
            Others + 
            Motor_Phase_Lck +
            Others_long +
            UCWP_Tail
)


print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("Message: %s" % message)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(message, (UDP_IP, UDP_PORT))