import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import pdb


class Yolo:
    def __init__(self):
        self.publisher = rospy.Publisher("frames", Image, queue_size=5)
        self.subscriber = rospy.Subscriber("camera", Image, self.callback, queue_size=5)

        self.bridge = CvBridge()

        self.classes = None

        # https://www.arunponnusamy.com/yolo-object-detection-opencv-python.html

        with open("coco_classes.txt", 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # generate different colors for different classes
        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        # read pre-trained model and config file
        self.net = cv2.dnn.readNet("../YOLO-LITE/cocoweights/trial6_342850.weights", "../YOLO-LITE/cococfg/trial6.cfg")

    def callback(self, frame):

        cv_frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding="passthrough")

        Width = cv_frame.shape[1]
        Height = cv_frame.shape[0]
        scale = 0.00392

        # create input blob
        blob = cv2.dnn.blobFromImage(cv_frame, scale, (416, 416), (0, 0, 0), True,  crop=False)

        # set input blob for the network
        self.net.setInput(blob)
        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        outs = self.net.forward(self.get_output_layers(self.net))
# for each detetion from each output layer
# get the confidence, class id, bounding box params
# and ignore weak detections (confidence < 0.5)
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

# apply non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

# go through the detections remaining
# after nms and draw bounding box
        for i in indices:
            i = i[0]
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]

            self.draw_bounding_box(cv_frame, class_ids[i], confidences[i], round(x), round(y), round(x + w), round(y + h))

        out_frame = self.bridge.cv2_to_imgmsg(cv_frame)

        self.publisher.publish(out_frame)

    def get_output_layers(self, net):

        layer_names = net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return output_layers

# function to draw bounding box on the detected object with class name

    def draw_bounding_box(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):

        label = str(self.classes[class_id])

        color = tuple(map(int, self.COLORS[class_id]))

        int_x = int(x)
        int_y = int(y)
        int_x_plus_w = int(x_plus_w)
        int_y_plus_h = int(y_plus_h)

        cv2.rectangle(img, (int_x, int_y), (int_x_plus_w, int_y_plus_h), color, 2)

        # pdb.set_trace()
        cv2.putText(img, label, (int_x-10, int_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    """ Initialises and cleans up after ros node"""

    yolo = Yolo()
    rospy.init_node("Yolo", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Exit")


if __name__ == '__main__':
    main()
