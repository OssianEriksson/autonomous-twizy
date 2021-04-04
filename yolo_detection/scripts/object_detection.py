#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from sensor_msgs.msg import Image
import numpy as np
from yolov4.tf import YOLOv4
import rospy
from std_msgs.msg import String
from yolo_detection.msg import BoundingBoxes,BoundingBox
import message_filters
from PIL import Image as plot_image
import numpy as np

yolo = YOLOv4() #initialize Yolo
#Remember to change weight source both for .cfg , .weights and coco.names
yolo.config.parse_names("src/yolo_detection/coco.names")                                    #coco.names list of object names
yolo.config.parse_cfg("src/yolo_detection/config/yolov4-tiny.cfg")                          # configure yolo with the config file 
yolo.make_model()                                                                           # creates a model (Don't know if needed 2021-03-31)
yolo.load_weights("src/yolo_detection/weights/yolov4-tiny.weights", weights_type="yolo")    # loads the yolo tiny weights,   
yolo.summary(summary_type="yolo")                                                           # summary? Don't know if needed 2021-03-31)

# Use cv_bridge() to convert the ROS image to OpenCV format
bridge = CvBridge()

def main():
    #Initialize ROS node
    rospy.init_node('listener', anonymous=True)

    prob_threshold = 0.6
    def convert_image(ros_image, boxes):
        try:
            #Convert the image using the default passthrough encoding
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

            temp_boxes = []
            for box in boxes.bounding_boxes: # Dim(-1, (x, y, w, h, cls_id, prob))  
                avreage = 0
                if box.ymax > -100 : 
                    for i in range(box.xmin, box.xmax): 
                        for j in range(box.ymin, box.ymax): 
                            avreage += cv_image[i][j]
                    avreage = avreage / ((box.xmax - box.xmin) * (box.ymax - box.ymin))    
                temp_boxes.append(avreage)

            print(temp_boxes)    
                
        except CvBridgeError as e: # error handling for CV
            print(e)     
    
    # subscribe to the image given from the realsense camera 
    aligned_camera = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, buff_size=10)
    boundingboxes  = message_filters.Subscriber('/boxes', BoundingBoxes, buff_size=10 )
    ts = message_filters.TimeSynchronizer([aligned_camera, boundingboxes], 10)
    ts.registerCallback(convert_image)
    try:
        rospy.spin() 
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()
    
