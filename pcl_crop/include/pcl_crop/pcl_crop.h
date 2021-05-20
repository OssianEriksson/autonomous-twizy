/**
 * @file pcl_crop.h
 *
 * @brief Contains classes used for cropping point clouds
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef PCL_CROP_PCL_CROP
#define PCL_CROP_PCL_CROP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

namespace pcl_crop {

/**
 * @brief Class representing a rectangular crop in two dimensions
 */
class Crop {
  public:
    /**
     * Left edge of crop as fraction of source width
     */
    double xmin = 0.0;

    /**
     * Right edge of crop as fraction of source width
     */
    double xmax = 1.0;

    /**
     * Top edge of crop as fraction of source height
     */
    double ymin = 0.0;

    /**
     * Bottom edge of crop as fraction of source height
     */
    double ymax = 1.0;

    /**
     * A value of true indicates that the interiour of the rectagle defined by
     * #xmin, #xmax, #ymin and #ymax is to be removed, false indicates that it
     * is the outside which is to be cropped out
     */
    bool interior = false;

    Crop(){};

    /**
     * @param x Horizontal position as fraction of source width of point to
     * investigate
     * @param y Vertical position as fraction of source height of point to
     * investigate
     *
     * @return true if the point (x, y) should be discarded as a result of this
     * crop, false otherwise
     */
    bool validate(double x, double y) const;
};

/**
 * @brief Class for cropping ordered ROS PointCloud2s
 */
class PclCrop {
  private:
    /**
     * ROS publisher used for publishing the cropped point cloud
     */
    ros::Publisher publisher_;

    /**
     * ROS subscriber used for subscribing to point clouds to crop
     */
    ros::Subscriber subscriber_;

    /**
     * List of all crops to perform
     */
    std::vector<Crop> crops_;

    /**
     * Points in the point cloud with z value less than this value are discarded
     */
    double zmin_ = 0.0;

    /**
     * Points in the point cloud with z value greater than this value are
     * discarded
     */
    double zmax_ = 10000.0;

    /**
     * Callback for #subscriber_
     */
    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  public:
    PclCrop(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
};

} // namespace pcl_crop

#endif