#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from rospkg import RosPack
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class TrackFace():
    def __init__(self):
        # Path where th classifier is
        self.bridge = CvBridge()
        rp = RosPack()
        self.pkg_path = rp.get_path('tracking_camera')
        self.haar_face_cascade = cv2.CascadeClassifier(self.pkg_path + "/src/nodes/haarcascade_frontalface_alt.xml")
        self.lbp_face_cascade = cv2.CascadeClassifier(self.pkg_path + '/src/nodes/lbpcascade_frontalface.xml')

        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        rospy.spin()

    def get_image_center(self, img):

        image_center=list(img.shape) #y x

        image_center_fin=list(image_center)
        image_center_fin[0]=float(image_center[1]/2)
        image_center_fin[1]=float(image_center[0]/2)

        #print(image_center_fin[0],image_center_fin[1])

        return image_center_fin

    def get_face_center(self, face):

        face_center=list()
        face_center=face[0] # face => x,y,w,h

        x=face_center[0]
        y=face_center[1]
        width=face_center[2]
        height=face_center[3]

        face_center[0]=x+height/2
        face_center[1]=y+width/2

        #print(face_center)
        return face_center

    def motors_command(self, face,img):

        coord_face = self.get_face_center(face)
        coord_center_img = self.get_image_center(img)
        print("Coord face:"+str(coord_face))
        print("Coord center img:"+str(coord_center_img))

        Px=1
        Py=1

        motors_command=list(coord_face)

        motors_command[0]=coord_face[0]-coord_center_img[0]*Px
        motors_command[1]=coord_face[1]-coord_center_img[1]*Py
        print("Motors Command:"+str(motors_command))
        return motors_command


    def convertToRGB(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def detect_faces(self, f_cascade, img, scaleFactor = 1.1):

    	# transfer RGB image to Gray scale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    	# Detecting all the face in the image and return the position in a list
        faces = f_cascade.detectMultiScale(gray, scaleFactor=scaleFactor, minNeighbors=5);

        return faces

    def draw_rectangle_on_face(self, img, faces):

    	# Draw a rectangle around each face of the list
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        return img

    def img_callback(self, img):

        image = self.bridge.imgmsg_to_cv2(img)

        faces_detected = self.detect_faces(self.lbp_face_cascade, image)
        faces_detected_img = self.draw_rectangle_on_face(image, faces_detected)

        # Display the resulting image
        cv2.imshow('frame',faces_detected_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise ROSInterruptException

        # Store the coordinates of faces in pose
        if len(faces_detected) > 0:
            for (x, y, w, h) in faces_detected:
                pose = "X : " + str(x) + " Y : " + str(y)

            self.motors_command(faces_detected, image)

        else:
            pose = None


if __name__ == "__main__":

    print("Face tracking started")
    rospy.init_node('track_face')

    try:
        tracking = TrackFace()
    except rospy.ROSInterruptException:
        pass
