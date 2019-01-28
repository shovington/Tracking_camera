#!/usr/bin/env python
import numpy as np
import cv2 
import rospy
from std_msgs.msg import String


def convertToRGB(img): 
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def detect_faces(f_cascade, img, scaleFactor = 1.1):          

	# transfer RGB image to Gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)          
	
	# Detecting all the face in the image and return the position in a list
    faces = f_cascade.detectMultiScale(gray, scaleFactor=scaleFactor, minNeighbors=5);           

    return faces

def draw_rectangle_on_face(img, faces):
    
	# Draw a rectangle around each face of the list
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)              

    return img

def talker(pose):
	# Function to publish the coordinate of the face on Face_pos topic
    pub = rospy.Publisher('Face_pos', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    str = pose
    rospy.loginfo(str)
    pub.publish(str)


def center_of_the_image(img):

    img_size = img.size

    # Determining the center of the image
    img_center[0] = img_size[0] / 2
    img_center[1] = img_size[1] / 2

    return img_center

def Motor_Command(center,pos_face):

    # Determining the motor command

    command=list()
    # Norm of the difference of the center of the face and the center of the image
    Norm = sqrt((center[0]-pos_face[0])**2+(center[1]-pos_face[1])**2)

    command[0]=center[0]-pos_face[0]/Norm
    command[1]=center[1]-pos_face[1]/Norm

    return command



def main():
	# Path where th classifier is
    haar_face_cascade = cv2.CascadeClassifier('home/catkin_ws/src/Tracking_camera/src/nodes/haarcascade_frontalface_alt.xml')
    lbp_face_cascade = cv2.CascadeClassifier('home/catkin_ws/src/Tracking_camera/src/nodes/lbpcascade_frontalface.xml')
 
	# 0 for webcam and 1 for usb cam
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Detect the faces and draw a rectangle around
        faces_detected = detect_faces(haar_face_cascade, frame)
        faces_detected_img = draw_rectangle_on_face(frame, faces_detected)

        # Display the resulting image
        cv2.imshow('frame',faces_detected_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        # Store the coordinates of faces in pose
        if len(faces_detected) > 0:
            for (x, y, w, h) in faces_detected:
                pose = "X : " + str(x) + " Y : " + str(y)
# + str(rospy.get_time())
				# Publish the coordinates on Face_pos topic
                talker(pose)

            center_of_the_image=center_of_the_image(frame)
            print(Motor_Command(center_of_the_image, pose))
        else:
            pose = "Detecting no face(s)"
        	# Publish the coordinates on Face_pos topic
            talker(pose)

if __name__ == "__main__":
    main()
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
