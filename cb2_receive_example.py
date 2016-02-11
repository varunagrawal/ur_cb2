import cb2_receive
import time
import socket

HOST = "192.168.1.100"    # The remote host
PORT = 30003              # The same port as used by the server

_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_socket.connect((HOST, PORT))
my_ur_receiver = cb2_receive.URReceiver(_socket, True)

my_ur_receiver.start()

# some_num = 0

try:
    while True:
        # print "\n\n" + str(some_num) + "\n\n"
        # some_num += 1
        time.sleep(.25)
except KeyboardInterrupt:
    my_ur_receiver.stop()
    pass
