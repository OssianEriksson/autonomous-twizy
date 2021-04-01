#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from sensor_msgs.msg import Image
import numpy as np
from yolov4.tf import YOLOv4
import rospy
from std_msgs.msg import String
from yolo_detection.msg import BoundingBoxes,BoundingBox

yolo = YOLOv4() #initialize Yolo
#Remember to change weight source both for .cfg , .weights and coco.names
yolo.config.parse_names("/home/ekalbin/autonomous-twizy/src/yolo_detection/coco.names")                 #coco.names list of object names
yolo.config.parse_cfg("/home/ekalbin/autonomous-twizy/src/yolo_detection/config/yolov4-tiny.cfg")       # configure yolo with the config file 
yolo.make_model()                                                                                       # creates a model (Don't know if needed 2021-03-31)
yolo.load_weights("/home/ekalbin/autonomous-twizy/src/yolo_detection/weights/yolov4-tiny.weights", weights_type="yolo") #loads the yolo tiny weights,   
yolo.summary(summary_type="yolo")                                                                       # summary? Don't know if needed 2021-03-31)


def main():
    #Initialize ROS node
    rospy.init_node('listener', anonymous=True)
    prob_threshold = 0.6
    # Use cv_bridge() to convert the ROS image to OpenCV format
    bridge = CvBridge()    
    def convert_image(ros_image):
        try:
            #Convert the image using the default passthrough encoding
            # cv_image = cv.fromarray(nparray())
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            #Convert the depth image to a Numpy array (not needed)
            #array_image = np.array(cv_image, dtype=np.float32)

            #assign the publisher what to publish
            box_pub = rospy.Publisher('boxes', BoundingBoxes , queue_size=10)
            image_pub = rospy.Publisher('/image_with_boxes', Image, queue_size=10)
            #rate = rospy.Rate(10) # 10hz
            # Because the msg is a "selfmade msg" of a "selfmade msg" we have to create a new instans of the msg.  
            msg = BoundingBoxes()
            # yolo.predict gives you an ndarry of the boxes therefore we remake them as bounding_box:es 
            #which we then add to the msg
            #global Boxes
            Boxes = yolo.predict(cv_image,prob_threshold) #returns  Dim(-1, (x, y, w, h, cls_id, prob))  
            image_with_boxes = yolo.draw_bboxes(cv_image, Boxes)
            ros_image_boxes = bridge.cv2_to_imgmsg(image_with_boxes, encoding="passthrough")
            image_pub.publish(ros_image_boxes)
           #cv.imshow('image',test)
           #cv.waitKey(1) 
            #print(Boxes)
            for box in Boxes:
               # print(box)
                temp = BoundingBox( box[5],box[0] - box[3]/2 ,box[1] - box[2]/2 ,box[0] + box[3]/2 ,box[1] - box[2]/2 ,int (box[4]), yolo.config.names[box[4]])
                msg.bounding_boxes.append(temp)
           
            # add msg header from the image and and one from yolo. 
            msg.image_header = ros_image.header
            msg.header.stamp = ros_image.header.stamp
            msg.header.frame_id = 'yolo'
            #publish msg
            box_pub.publish(msg) 
        except CvBridgeError as e: # error handling for CV
            print(e)

    # subscribe to the image given from the realsense camera 
    rospy.Subscriber('/camera/color/image_raw', Image, convert_image, queue_size=1)
    try:
        rospy.spin() 

    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()
    
