#include "pcl_crop/pcl_crop.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace pcl_crop {

PclCrop::PclCrop(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {
    XmlRpc::XmlRpcValue crops;
    nh_private.getParam("crops", crops);
    ROS_ASSERT(crops.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < crops.size(); i++) {
        Crop c;
        if (crops[i].hasMember("xmin")) {
            c.xmin = crops[i]["xmin"];
        }
        if (crops[i].hasMember("xmax")) {
            c.xmax = crops[i]["xmax"];
        }
        if (crops[i].hasMember("ymin")) {
            c.ymin = crops[i]["ymin"];
        }
        if (crops[i].hasMember("ymax")) {
            c.ymax = crops[i]["ymax"];
        }
        if (crops[i].hasMember("interior")) {
            c.interior = crops[i]["interior"];
        }
        crops_.push_back(c);
    }

    nh_private.getParam("zmin", zmin_);
    nh_private.getParam("zmax", zmax_);

    publisher_ = nh.advertise<sensor_msgs::PointCloud2>("cropped", 1);
    subscriber_ = nh.subscribe("image", 1, &PclCrop::callback, this);
}

void PclCrop::callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::PointCloud2 points;
    points.header = msg->header;
    points.is_dense = true;
    points.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier pcd_modifier(points);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(msg->width * msg->height);

    sensor_msgs::PointCloud2Iterator<float> iter_x(points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(points, "z");

    int i = 0;
    int valid_points = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x");
         it != it.end(); ++it, ++i) {
        if (it[2] > zmax_ || it[2] < zmin_) {
            continue;
        }

        double x = (i % msg->width + 0.5) / msg->width;
        double y = (i / msg->width + 0.5) / msg->height;
        for (auto const &crop : crops_) {
            if (!crop.validate(x, y)) {
                goto skip_point;
            }
        }

        *iter_x = it[0];
        *iter_y = it[1];
        *iter_z = it[2];

        ++iter_x;
        ++iter_y;
        ++iter_z;

        valid_points++;

    skip_point:;
    }

    points.height = 1;
    points.width = valid_points;
    pcd_modifier.resize(valid_points);

    publisher_.publish(points);
}

bool Crop::validate(double x, double y) const {
    return interior != (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
}

} // namespace pcl_crop
