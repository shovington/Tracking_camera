#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from rospkg import RosPack
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CompressedImage
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError

class TrackPerson():
    def __init__(self):
        # Path where th classifier is
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/darknet_ros/detection_image",Image, self.img_callback) #Subscriber(topic, condition, to do)
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.box_callback)
        rospy.spin()

    def img_callback(self, img):

        image = self.bridge.imgmsg_to_cv2(img)
        #print(image.shape)
        # Display the resulting image
        cv2.imshow('frame',image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise ROSInterruptException

        else:
            pose = None

    def box_callback(self, box): #We take the first index which is the one who has the most probability

        center_image=list()
        center_image=[320,240]

        center_box=list()
        command=list()

        for i in range(len(box.bounding_boxes)):

            if box.bounding_boxes[i].Class=="person":

                center_box.append((box.bounding_boxes[i].xmin+box.bounding_boxes[i].xmax)/2)
                center_box.append((box.bounding_boxes[i].ymin+box.bounding_boxes[i].ymax)/2)

                command.append(center_box[0]-center_image[0])
                command.append(center_box[1]-center_image[1])

                break

        print(command)
        return command


if __name__ == "__main__":

    print("Face tracking started")
    rospy.init_node('track_face')

    try:
        tracking = TrackPerson()
    except rospy.ROSInterruptException:
        pass