import socket


#################
#               #
#   VARIABLES   #
#               #
#################


UDP_IP = "127.0.0.1"
UDP_UCWP_PORT = 5005
bufferSize = 1248*2



#####################
#                   #
#   MAIN PROGRAM    #
#                   #
#####################

print("#################################")
print("#    Launched UCWPsniffer.py    #")
print("#################################")

# UDP socket bind
sock_UCWP = socket.socket(socket.AF_INET,    # Internet
                          socket.SOCK_DGRAM) # UDP
sock_UCWP.bind((UDP_IP, UDP_UCWP_PORT))

print("Waiting for message...")
while True:
    # Data fetching on socket
    data_byte, addr = sock_UCWP.recvfrom(bufferSize)
    #data_hex = data_byte.hex()
    print("UCWP message: %s" % data_byte)