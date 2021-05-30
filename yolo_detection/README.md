# yolo_detection <!-- omit in toc -->

ROS wrapper for the [YOLOv4](https://arxiv.org/abs/2004.10934) object detection algorithm, by default trained on the [COCO dataset](https://cocodataset.org/), using a [python implementation of yolov4](https://pypi.org/project/yolov4/).

This package requires the installation of the python package yolov4 (`pip3 install yolov4`) and it's dependencies which you can read about [here](https://wiki.loliot.net/docs/lang/python/libraries/yolov4/python-yolov4-about/#dependencies).

**Package Links**

* [Msg/Srv API](https://ossianeriksson.github.io/autonomous-twizy/yolo_detection/html/index-msg.html)

**Contents**

- [Nodes](#nodes)
  - [bounding_box_calc](#bounding_box_calc)
    - [Subscribed Topics](#subscribed-topics)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)

# Nodes

## bounding_box_calc

Inferes bounding boxes around detected objects from the COCO dataset in input image messages.

### Subscribed Topics

* `image{N}` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))  
  Input raw color images to apply the yolo algoritm on. `N` is an integer from 1 up to (and including) the value of the parameter `~num_image_sources`. All these topics will be subscribed to and processed in parallel.

### Published Topics

* `image_with_boxes{N}` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))  
  Input images with inferred bounding boxes drawn on top of them for visualization purposes. `N` is an integer from 1 up to (and including) the value of the parameter `~num_image_sources`. Each topic corresponds to an input image topic `image{N}`.

* `boxes{N}` ([yolo_detection/BoundingBoxes](msg/BoundingBoxes.msg))  
  Lists of bounding boxes inferred from input images. `N` is an integer from 1 up to (and including) the value of the parameter `~num_image_sources`. Each topic corresponds to an input image topic `image{N}`.

### Parameters

* `~num_image_sources` (`int`, required)  
  Number of parallel image sources and outputs to subscribe to/publish. Must be greater than or equal to `1`.

* `~prob_threshold` (`double`, default: `0.6`)  
  Probility threashold: Infered bounding boxes with an estimated probability of correct classification below this value are discarded and not included in the output.

* `~names` (`string`, default: `"<package folder>/coco.names"`)  
  Path to a file containg a list of names of ordered classes which the network recognizes. For an example of such a file see the default one, [coco.names](coco.names).

* `~cfg` (`string`, default: `"<package folder>/config/yolov4-tiny.cfg"`)  
  Path to a file containg a network configuration to be read by the yolov4 python package which in turn uses tensorflow.

* `~weight` (`string`, default: `"<package folder>/weights/yolov4-tiny.weights"`)  
  Path to a file containg pre-trained YOLO weights to be read by the yolov4 python package which in turn uses tensorflow. The weights must correspond to the network configuration selected with `~cfg` and the class names determined by `~names`.
