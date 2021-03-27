#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from yolov4.tf import YOLOv4
import rospy
from std_msgs.msg import String

prob_threshold = float(0.6)
yolo = YOLOv4()
array_image = 0
cv_image = 0

yolo.config.parse_names("src/yolo_detection/coco.names")
yolo.config.parse_cfg("src/yolo_detection/config/yolov4-tiny.cfg")   #Remember to change weight source both for .cfg , .weights and coco.names
yolo.make_model()
yolo.load_weights("src/yolo_detection/weights/yolov4-tiny.weights", weights_type="yolo")       
yolo.summary(summary_type="yolo")


def convert_image(ros_image):
    global array_image
    global cv_image
    # Use cv_bridge() to convert the ROS image to OpenCV format
    bridge = CvBridge()
    try:
        #Convert the  image using the default passthrough encoding
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        #Convert the depth image to a Numpy array (not needed)
        #array_image = np.array(cv_image, dtype=np.float32)
        talker()
    except CvBridgeError as e:
        print(e) 
    

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, convert_image, queue_size=1)
    rospy.spin()    

def talker():
    global array_image
    global cv_image
    global prob_threshold
    pub = rospy.Publisher('boxes', String , queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        Boxes = yolo.predict(cv_image,0.2)
        # rospy.loginfo(Boxes)
        # pub.publish(Boxes)
        # rate.sleep()

if __name__ == '__main__':
    listener()
    