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
    /**
     * Pointer to the class which contains all features of this node. A
     * reference is kept so the memory is not released after the
     * PclCropNodelet::onInit() returns
     */
    std::unique_ptr<PclCrop> crop_;

    virtual void onInit() {
        // Get references to node handles
        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &nh_private = getPrivateNodeHandle();

        // Start cropping point clouds
        crop_ = std::unique_ptr<PclCrop>(new PclCrop(nh, nh_private));
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(pcl_crop::PclCropNodelet, nodelet::Nodelet)
} // namespace pcl_crop