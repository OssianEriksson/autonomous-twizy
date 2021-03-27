#ifndef ACKERMANN_EKF_SENSOR_ARRAY
#define ACKERMANN_EKF_SENSOR_ARRAY

#include "ackermann_ekf_cpp/ackermann_ekf.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <string>

namespace ackermann_ekf
{
    class SensorArray
    {
    private:
        AckermannEkf filter_;

        ros::NodeHandle nh_;

        ros::NodeHandle nh_private_;

        tf2_ros::Buffer tf_buffer_;

        tf2_ros::TransformListener tf_listener_;

        std::string base_link_;

        float frequency_;

    public:
        SensorArray(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private);
        
        void initialize();

        bool update_transform(const std::string &frame_id,
                              const ros::Time &time,
                              geometry_msgs::TransformStamped &transform);
    };
}

#endif