#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define a callback function to handle the received image messages
def image_callback(msg):
    # Create a CvBridge object
    bridge = CvBridge()
    try:
        # Convert the ROS image message to OpenCV's image format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        # If there is a conversion error, print the error message and return
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Load the Haar Cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')
    # Convert the image to grayscale for face detection
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Use the cascade classifier to detect faces on the grayscale image
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # For each detected face, draw a blue rectangle on the original image
    for (x, y, w, h) in faces:
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        # Crop and save the detected face region image
        face_img = cv_image[y:y+h, x:x+w]
        cv2.imwrite('detected_face.jpg', face_img)

    # Display the processed image in a window
    cv2.imshow("Image Window", cv_image)
    # Wait 3 milliseconds before refreshing the image window
    cv2.waitKey(3)

# Main function
def main():
    # Initialize a ROS node named 'face_detector'
    rospy.init_node('face_detector', anonymous=True)
    # Subscribe to the '/camera/color/image_raw' topic to receive image messages
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    # Keep the node running until manually shut down
    rospy.spin()

# Program entry point
if __name__ == '__main__':
    main()

