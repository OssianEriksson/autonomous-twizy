#!/usr/bin/env python

import rospy
import cv2 as cv
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolo_detection.msg import BoundingBoxes,BoundingBox


#Use cv_bridge() to convert the ROS image to OpenCV format
bridge = CvBridge()
def main():
    #Initialize ROS node
    rospy.init_node('listener', anonymous=True)
    
    # callback function
    def convert_image(ros_image, boxes):
        try:
            #Convert the image using the default passthrough encoding
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

            # create an array for the distance,object tuple
            distance_to_object = []

            #loops over every box and calculate the avreage distance to each object by adding the distance to every pixel
            # when we don't find any object the camera returns ymax as -106 there for we check before going inside the second loop.
            for box in boxes.bounding_boxes:  
                avreage = 0
                if box.ymax > -100 : 
                    for i in range(box.xmin, box.xmax): 
                        for j in range(box.ymin, box.ymax): 
                            avreage += cv_image[j][i]
                    avreage = avreage / ((box.xmax - box.xmin) * (box.ymax - box.ymin))    
                distance_to_object.append((avreage,box.Class))

            #add publisher?
            #print(distance_to_object)    
                
        except CvBridgeError as e: # error handling for CV
            print(e)     
    

    #   subscribe to the image given from the realsense camera and the boundingboxes given from yolo
    #   then aligne the messages so you get the rigth boxes to the rigth image.

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
    
