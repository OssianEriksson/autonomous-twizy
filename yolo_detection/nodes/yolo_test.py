#!/usr/bin/python3

import cv2

from yolov4.tf import YOLOv4
yolo = YOLOv4()


yolo.config.parse_names("src/yolo_detection/coco.names")
#yolo.config.parse_cfg("src/yolo_detection/config/yolov4-tiny.cfg")   #Remember to change weight source both for .cfg and .weights
yolo.config.parse_cfg("src/yolo_detection/config/yolov4-rds-colab.cfg")
yolo.make_model()
#yolo.load_weights("src/yolo_detection/weights/yolov4-tiny.weights", weights_type="yolo")
yolo.load_weights("/home/ekalbin/autonomous-twizy/src/yolo_detection/weights/yolov4-rds_best_2000.weights")
yolo.summary(summary_type="yolo")

yolo.inference(media_path="/home/ekalbin/autonomous-twizy/src/yolo_detection/test/test.jpg")

##yolo.inference(media_path="road.mp4", is_image=False)

#To find camera sources available, run ls -ltrh /dev/video*


# yolo.inference(
#    "/dev/video6",             #video0 is webcam, video6 is realsense RGB
#    is_image=False,
#    cv_apiPreference=cv2.CAP_V4L2,
#    cv_frame_size=(640, 480),
#    cv_fourcc="YUYV",
#    cv_waitKey_delay=10,
#    prob_thresh=0.2,  
# )