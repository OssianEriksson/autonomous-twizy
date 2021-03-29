#include "ackermann_ekf_cpp/sensor_array.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace ackermann_ekf {
SensorArray::SensorArray(const ros::NodeHandle &nh,
                         const ros::NodeHandle &nh_private)
    : filter_(), nh_(nh), nh_private_(nh_private),
      tf_buffer_(*new tf2_ros::Buffer),
      tf_listener_(*new tf2_ros::TransformListener(tf_buffer_)),
      base_link_("base_link"), frequency_(30.0) {}

void SensorArray::initialize() {
  nh_private_.getParam("base_link", base_link_);
  nh_private_.getParam("frequency", frequency_);

  std::cout << base_link_ << "  " << frequency_ << std::endl;
  std::cout << nh_private_.resolveName("sensors") << std::endl;

  XmlRpc::XmlRpcValue sensors;
  nh_private_.getParam("sensors", sensors);
  ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < sensors.size(); i++) {
    if (sensors[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      continue;
    }

    std::string topic = sensors[i]["topic"];
    std::string type = sensors[i]["type"];

    if (topic.empty() || type.empty()) {
      ROS_WARN("Missing either topic or type for sensor");
      continue;
    }

    std::array<bool, MEASUREMENT_SIZE> mask;
    if (sensors[i].hasMember("mask")) {
      XmlRpc::XmlRpcValue mask_list = sensors[i]["mask"];
      ROS_ASSERT(mask_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int j = 0; j < MEASUREMENT_SIZE; j++) {
        mask[j] = static_cast<bool>(mask_list[j]);
      }
    } else {
      for (int j = 0; j < MEASUREMENT_SIZE; j++) {
        mask[j] = true;
      }
    }

    if (type == "sensor_msgs/NavSatFix") {
      std::cout << "Creating NavSatFix sensor" << std::endl;
      NavSatFixSensor(*this, mask);
    }
  }
}

bool SensorArray::update_transform(const std::string &frame_id,
                                   const ros::Time &time,
                                   geometry_msgs::TransformStamped &transform) {
  try {
    transform = tf_buffer_.lookupTransform(base_link_, frame_id, time);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}
} // namespace ackermann_ekf