import socket
import rospy
import numpy as np
import libh264decoder
from sensor_msgs.msg import Image
import pdb
import threading


class Video_Feed:
    def __init__(self):

        self.message_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.message_address = ("192.168.10.1", 8899)
        self.video_port = 11111
        self.decoder = libh264decoder.H264Decoder()

        # "" represents INADDR_ANY and is used to bind to all interfaces
        rospy.init_node("raw_video", anonymous=True)
        self.publisher = rospy.Publisher("raw_video", Image, queue_size=1)

        self.message_socket.sendto(b"command", self.message_address)
        print("sent: command")
        self.message_socket.sendto(b"streamon", self.message_address)
        print("sent: streamon")

        self.video_socket.bind(("", self.video_port))

        recieve_thread = threading.Thread(target=self.recieve)
        recieve_thread.daemon = True
        recieve_thread.start()

        # pdb.set_trace()

    def recieve(self):
        packet_data = ""
        while True:
            try:
                res_string, ip = self.video_socket.recvfrom(2048)
                pdb.set_trace()
                packet_data += res_string
                if len(res_string) != 1460:
                    for frame in self._h264_decode(packet_data):
                        # pdb.set_trace()
                        rospy.loginfo("published frame")
                        self.publisher.publish(frame)

                    packet_data = ""
            except socket.error as exc:
                print("Cautch exception TypeError {}".format(exc))

    def _h264_decode(self, packet_data):
        pdb.set_trace()
        """
        decode raw h264 format data from Tello

        :param packet_data: raw h264 data array

        :return: a list of decoded frame
        """
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                print('frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls))

                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, ls / 3, 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list


def main():
    """ Initialises and cleansup after ros node"""
    feed = Video_Feed()
    try:
        feed.recieve()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("down we go")


if __name__ == '__main__':
    main()
