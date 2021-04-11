#ifndef TWIZY_WHEEL_ENCODER_DEFINITIONS
#define TWIZY_WHEEL_ENCODER_DEFINITIONS

#include <ros.h>

#define MAX_SUBSCRIBERS 1
#define MAX_PUBLISHERS 2
#define INPUT_SIZE 150
#define OUTPUT_SIZE 150

namespace twizy_wheel_encoder {
typedef ros::NodeHandle_<ArduinoHardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS,
                         INPUT_SIZE, OUTPUT_SIZE>
    NodeHandle;

struct Parameters {
    struct Side {
        char frame_id[24];
    };

    float covariance;
    float sensitivity;
    unsigned int mspt = 0;
    Side left, right;
};
} // namespace twizy_wheel_encoder

#endif