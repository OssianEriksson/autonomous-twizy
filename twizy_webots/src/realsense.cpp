#include "twizy_webots/realsense.h"

#include <iostream>
#include <limits>
#include <math.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace twizy_webots {

Realsense::Realsense(webots::Supervisor &supervisor, ros::NodeHandle &nh,
                     std::string position)
    : camera_(supervisor.getCamera(position + "_realsense_color_image_raw")),
      range_finder_(supervisor.getRangeFinder(
          position + "_realsense_aligned_depth_to_color")),
      position_(position) {

    std::cout << "Initializing " << position << " RealSense camera\n";

    double ups = 30.0;

    camera_->enable(round(1000.0 / ups));
    range_finder_->enable(round(1000.0 / ups));

    pub_camera_ = nh.advertise<sensor_msgs::Image>(
        "/camera/" + position + "/color/image_raw", 1);
    pub_camera_info_ = nh.advertise<sensor_msgs::CameraInfo>(
        "/camera/" + position + "/color/camera_info", 1);
    pub_range_ = nh.advertise<sensor_msgs::Image>(
        "/camera/" + position + "/aligned_depth_to_color/image_raw", 1);
    pub_range_info_ = nh.advertise<sensor_msgs::CameraInfo>(
        "/camera/" + position + "/aligned_depth_to_color/camera_info", 1);
    pub_points_ = nh.advertise<sensor_msgs::PointCloud2>(
        "/camera/" + position + "/depth/color/points", 1);
    pub_cropped_points_ = nh.advertise<sensor_msgs::PointCloud2>(
        "/camera/" + position + "/depth/color/cropped/points", 1);

    update_timer_ =
        nh.createTimer(ros::Duration(1.0 / ups), &Realsense::update, this);
}

void Realsense::update(const ros::TimerEvent &evt) {
    ROS_ASSERT(camera_->getWidth() == range_finder_->getWidth() &&
               camera_->getHeight() == range_finder_->getHeight());

    ros::Time stamp = ros::Time::now();

    const unsigned char *camera_data = camera_->getImage();
    sensor_msgs::Image camera_image;
    camera_image.header.stamp = stamp;
    camera_image.header.frame_id = "camera/" + position_ + "_link";
    camera_image.height = camera_->getHeight();
    camera_image.width = camera_->getWidth();
    camera_image.encoding = sensor_msgs::image_encodings::BGRA8;
    camera_image.step = sizeof(char) * 4 * camera_->getWidth();
    camera_image.data.resize(4 * camera_->getWidth() * camera_->getHeight());
    memcpy(&camera_image.data[0], camera_data,
           sizeof(char) * 4 * camera_->getWidth() * camera_->getHeight());

    sensor_msgs::CameraInfo camera_info;
    camera_info.header = camera_image.header;
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.width = camera_->getWidth();
    camera_info.height = camera_->getHeight();
    double f = 0.5 * camera_->getWidth() / tan(0.5 * camera_->getFov());
    double cx = camera_->getWidth() * 0.5;
    double cy = camera_->getHeight() * 0.5;
    camera_info.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    camera_info.P = {f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    const float *range_data;
    range_data = (float *)(void *)range_finder_->getRangeImage();
    sensor_msgs::Image range_image;
    range_image.header = camera_image.header;
    range_image.height = range_finder_->getHeight();
    range_image.width = range_finder_->getWidth();
    range_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    range_image.step = 2 * range_finder_->getWidth();
    range_image.data.resize(2 * range_finder_->getWidth() *
                            range_finder_->getHeight());

    sensor_msgs::PointCloud2 points;
    points.header = range_image.header;
    points.height = camera_->getHeight();
    points.width = camera_->getWidth();
    points.is_dense = false;
    points.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier pcd_modifier(points);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    uint16_t *processed_range_data =
        (uint16_t *)(void *)range_image.data.data();
    sensor_msgs::PointCloud2Iterator<float> iter_x(points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(points, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(points, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(points, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(points, "b");
    for (int v = 0; v < camera_->getHeight(); v++) {
        for (int u = 0; u < camera_->getWidth(); u++, processed_range_data++,
                 camera_data += 4, range_data++, ++iter_x, ++iter_y, ++iter_z,
                 ++iter_r, ++iter_g, ++iter_b) {
            *processed_range_data = round(*range_data * 1000.0);

            *iter_x = *range_data;
            *iter_y = *range_data * (cx - u) / f;
            *iter_z = *range_data * (cy - v) / f;
            *iter_r = camera_data[2];
            *iter_g = camera_data[1];
            *iter_b = camera_data[0];
        }
    }

    pub_camera_.publish(camera_image);
    pub_camera_info_.publish(camera_info);
    pub_range_.publish(range_image);
    pub_range_info_.publish(camera_info);
    pub_points_.publish(points);
    pub_cropped_points_.publish(points);
}

Realsense::~Realsense() {
    camera_.reset();
    range_finder_.reset();
}

} // namespace twizy_webots