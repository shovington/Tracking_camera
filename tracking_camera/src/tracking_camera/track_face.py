#!/usr/bin/env python

import numpy as np
import rospy
from rospkg import RosPack
from std_msgs.msg import String, Int32
from darknet_ros_msgs.msg import BoundingBoxes

class TrackFace():
    def __init__(self):
	pass

    def box_offset(self, box): #We take the first index only which is the one who has the most probability

        center_image=list()
        center_image=[320,240]

        center_box=list()
        command=list()

        for i in range(len(box.bounding_boxes)):

            if box.bounding_boxes[i].Class=="person":

                center_box.append((box.bounding_boxes[i].xmin+box.bounding_boxes[i].xmax)/2)
                center_box.append((box.bounding_boxes[i].ymin+box.bounding_boxes[i].ymax/3))

                command.append(center_box[0]-center_image[0])
                command.append(center_box[1]-center_image[1])

                break

        return command


if __name__ == "__main__":

    print("Face tracking started")
    rospy.init_node('track_face')

    try:
        tracking = TrackPerson()
    except rospy.ROSInterruptException:
        pass
