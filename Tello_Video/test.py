import socket
test = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

address = ("192.168.10.1", 8889)


test.sendto(b"command", address)
test.sendto(b"streamon", address)

while True:
    print(test.sendto(b"battery?", address))


