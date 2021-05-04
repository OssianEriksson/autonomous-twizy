#include "pcl_crop/pcl_crop.h"

#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace pcl_crop {
class PclCropNodelet : public nodelet::Nodelet {
  public:
    ~PclCropNodelet() { crop_.reset(); }

  private:
    std::unique_ptr<PclCrop> crop_;

    virtual void onInit() {
        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &nh_private = getPrivateNodeHandle();

        crop_ = std::unique_ptr<PclCrop>(new PclCrop(nh, nh_private));
    }
};

PLUGINLIB_EXPORT_CLASS(pcl_crop::PclCropNodelet, nodelet::Nodelet)
} // namespace pcl_crop