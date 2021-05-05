#ifndef PCL_CROP_PCL_CROP
#define PCL_CROP_PCL_CROP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

namespace pcl_crop {

class Crop {
  public:
    double xmin = 0.0, xmax = 1.0, ymin = 0.0, ymax = 1.0;
    bool interior = false;

    Crop(){};

    bool validate(double x, double y) const;
};

class PclCrop {
  private:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    std::vector<Crop> crops_;

    double zmin_ = 0.0, zmax_ = 10000.0;

    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  public:
    PclCrop(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
};

} // namespace pcl_crop

#endif