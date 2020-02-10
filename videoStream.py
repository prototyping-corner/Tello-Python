import cv2
import socket
import numpy as np

UDP_IP = '192.168.10.1'
UDP_PORT = 8889
RESPONSE_TIMEOUT = 7  # in seconds
TIME_BTW_COMMANDS = 1  # in seconds
TIME_BTW_RC_CONTROL_COMMANDS = 0.5  # in seconds
RETRY_COUNT = 3

VS_UDP_IP = '0.0.0.0'
VS_UDP_PORT = 11111

clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
clientSocket.bind(('0.0.0.0', UDP_PORT))

def get_udp_video_address():
    return 'udp://@' + VS_UDP_IP + ':' + str(VS_UDP_PORT)  # + '?overrun_nonfatal=1&fifo_size=5000'

def send_control_command(command, timeout=RESPONSE_TIMEOUT):
    """Send control command to Tello and wait for its response. Possible control commands:
        - command: entry SDK mode
         - takeoff: Tello auto takeoff
            - land: Tello auto land
            - streamon: Set video stream on
            - streamoff: Set video stream off
            - emergency: Stop all motors immediately
            - up x: Tello fly up with distance x cm. x: 20-500
            - down x: Tello fly down with distance x cm. x: 20-500
            - left x: Tello fly left with distance x cm. x: 20-500
            - right x: Tello fly right with distance x cm. x: 20-500
            - forward x: Tello fly forward with distance x cm. x: 20-500
            - back x: Tello fly back with distance x cm. x: 20-500
            - cw x: Tello rotate x degree clockwise x: 1-3600
            - ccw x: Tello rotate x degree counter- clockwise. x: 1-3600
            - flip x: Tello fly flip x
                l (left)
                r (right)
                f (forward)
                b (back)
            - speed x: set speed to x cm/s. x: 10-100
            - wifi ssid pass: Set Wi-Fi with SSID password
        Return:
            bool: True for successful, False for unsuccessful
        """
#    response = None
#    for i in range(0, RETRY_COUNT):
#        response = send_command_with_return(command, timeout=timeout)
#        if response == 'OK' or response == 'ok':
#            return True

    #return
 #   return response
    clientSocket.sendto(command.encode('utf-8'), ('192.168.10.1', 8889))
    return "assume"

address = get_udp_video_address()

print("Connect: ")
print(send_control_command("command"))
print("Streamon: ")
print(send_control_command("streamon"))

cap = cv2.VideoCapture(address)

if not cap.isOpened():
    cap.open(addrss)

grabbed, raw_frame = cap.read()

stopped = False

while not stopped:
    if not grabbed or not cap.isOpened():
        stopped = True
    else:
        (grabbed, raw_frame) = cap.read()
        #frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
        #frame = np.rot90(frame)
        #frame = np.flipud(frame)
        cv2.imshow('output', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

print("End")
