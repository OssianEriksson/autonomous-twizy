#ifndef TWIZY_WHEEL_ENCODER_DEFINITIONS
#define TWIZY_WHEEL_ENCODER_DEFINITIONS

#include <ros.h>

#define MAX_SUBSCRIBERS 1
#define MAX_PUBLISHERS 2
#define INPUT_SIZE 150
#define OUTPUT_SIZE 150

namespace twizy_wheel_encoder {

// Customizing the NodeHandle decreases memory usage
typedef ros::NodeHandle_<ArduinoHardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS,
                         INPUT_SIZE, OUTPUT_SIZE>
    NodeHandle;

/**
 * @brief Collection of parameters to the wheel encoder ROS node
 */
struct Parameters {
    /**
     * @brief Parameters specific to the weel encoders on each side of the
     * vehicle
     */
    struct Side {
        /**
         * frame_id corresponding to position of the wheel
         */
        char frame_id[24];
    };

    /**
     * Speed convariance
     */
    float covariance;

    /**
     * Conversion factor between encoder ticks per ms and m/s
     */
    float sensitivity;

    /**
     * Milliseconds to delay between publishings of sensor values
     */
    unsigned int mspt = 0;

    /**
     * Parameters corresponding to left wheel encoder
     */
    Side left;

    /**
     * Parameters corresponding to right wheel encoder
     */
    Side right;
};

} // namespace twizy_wheel_encoder

#endif