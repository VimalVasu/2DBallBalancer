#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

import numpy as np

# Initialize ROS node
rospy.init_node('green_ball_detector')

# Initialize CvBridge
bridge = CvBridge()

# Publisher for x-coordinate
pos_pub = rospy.Publisher('green_ball_coords', Point, queue_size=10)

# Open a handle to the default webcam
cap = cv2.VideoCapture(0)

# Reduce the capture resolution to increase frame rate
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Disable auto-exposure and auto-white balance if supported
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 for manual exposure
cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # 0 for manual white balance

if not cap.isOpened():
    rospy.logerr("Cannot open camera")
    exit()

# Define the range of green color in HSV
lower_green = np.array([35, 40, 40])  # Adjust these values based on your green object
upper_green = np.array([85, 255, 255])  # Adjust these values based on your green object
#green: 40,40,40 80,255,255?
# Capture frame-by-frame
while not rospy.is_shutdown():
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        rospy.logerr("Can't receive frame (stream end?). Exiting ...")
        break

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the green color
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    for contour in contours:
        # Calculate the area of each contour
        area = cv2.contourArea(contour)
        # Only proceed if the area is greater than a certain threshold to avoid noise
        if area > 500:
            # Get the bounding circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            # Create Point message
            point = Point()
            point.x = float(x) - 320.0
            point.y = float(y) - 240.0
            point.z = 0.0
            # Publish coordinates
            pos_pub.publish(point)
		
		
	    # Draw the circle around the green ball
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 255, 0), 2)  # Green circle with thickness 2

    # Resize frame before displaying to decrease window size
    frame_resized = cv2.resize(frame, (420, 340))  # Resize to 320x240 for the display

    # Display the resulting frame
    cv2.imshow('frame', frame_resized)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

