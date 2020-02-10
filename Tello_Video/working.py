import socket
import pdb

message = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tello_address = ("192.168.10.1", 8889)

video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

message.sendto(b"command", tello_address)
message.sendto(b"streamon", tello_address)

video.bind(("", 11111))

test = video.recvfrom(2048)
pdb.set_trace()
