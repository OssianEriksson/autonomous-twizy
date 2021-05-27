#include "twizy_webots/velodyne.h"

#include <cmath>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace twizy_webots {

Velodyne::Velodyne(webots::Supervisor &supervisor, ros::NodeHandle &nh)
    : lidar_(supervisor.getLidar("lidar_vlp16_webots")) {

    std::cout << "Initializing Velodyne VLP16 LiDAR\n";

    // Constant hard coded update rate
    double ups = 20.0;

    // Enable sensors
    lidar_->enable(round(1000.0 / ups));
    lidar_->enablePointCloud();

    publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar/points", 1);

    update_timer_ =
        nh.createTimer(ros::Duration(1.0 / ups), &Velodyne::update, this);
}

void Velodyne::update(const ros::TimerEvent &evt) {
    const WbLidarPoint *pc = lidar_->getPointCloud();
    sensor_msgs::PointCloud2 points;
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "lidar_vlp16_webots";
    points.height = 1;
    points.width = lidar_->getNumberOfPoints();
    points.is_dense = false;
    points.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier pcd_modifier(points);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(points, "z");
    double last_distance = 1.0;
    int valid_points = 0;
    for (int i = 0; i < points.width; i++) {
        if (std::isfinite(pc[i].x)) {
            // Estimate the distance of this point from the LiDAR sensor
            double distance = abs(pc[i].x) + abs(pc[i].y) + abs(pc[i].z);
            // Distance from the LiDAR sensor of this point compared to the
            // previous
            double ratio = distance / last_distance;

            // This filtering is done to reduce a problem with the way Webots
            // samples its depth textures: The sampling is not e.g. nearest
            // neighbor but maybe linear or something like it which basically
            // blurs the depth image when accessing values between pixels. If
            // you imagine having two walls beside each other, one 50 m and one
            // 100 m away from the camera, this blur could mean that values
            // other than 50 or 100 m are reported on the edge between the two
            // distances. This results in unwanted point cloud points. To reduce
            // this effect we require that neighbouring points in the point
            // cloud must be a similar distance from the camera or be discarded.
            // This way we e.g. let through the bunches of 50 m points and the
            // bunches of 100 m points but discard the few points which have
            // values ranging between 50 and 100 m. This is still a hacky
            // solution since the point cloud returned by Webots is not
            // guarranteed to be organized (points could come in any order) but
            // it works for now. (Take care when sampling depth textures kids!)
            if (ratio > 0.99 && ratio < 1.01) {
                *iter_x = pc[i].x;
                *iter_y = pc[i].y;
                *iter_z = pc[i].z;

                ++iter_x;
                ++iter_y;
                ++iter_z;

                valid_points++;
            }
            last_distance = distance + 1e-3;
        }
    }

    points.width = valid_points;
    pcd_modifier.resize(valid_points);

    publisher_.publish(points);
}

Velodyne::~Velodyne() { lidar_.reset(); }

} // namespace twizy_webots