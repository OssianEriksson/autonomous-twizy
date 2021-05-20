#ifndef TWIZY_WHEEL_ENCODER_SENSOR
#define TWIZY_WHEEL_ENCODER_SENSOR

#include "definitions.h"

#include <twizy_wheel_encoder/WheelEncoder.h>

namespace twizy_wheel_encoder {

/**
 * @brief Class representing a wheel encoder sensor
 */
class Sensor {
  private:
    /**
     * ROS message to be published by this sensor
     */
    WheelEncoder msg_;

    /**
     * Parameters for this sensor common to all wheel encoder sensors
     */
    const Parameters &params_;

    /**
     * Parameters for this sensor specific to this wheel encoder sensor
     */
    const Parameters::Side &side_;

    /**
     * Last time (ms) an encoder tick was recorded
     */
    unsigned long last_time_;

  public:
    /**
     * Publisher of WheelEncoder messages
     */
    ros::Publisher publisher;

    Sensor(const char *topic_name, const Parameters &params,
           const Parameters::Side &side);

    /**
     * Initializes this sensor
     *
     * @param nh Publish node handle of a ROS node
     * @param pin Index of arduino pin connected to the encoder
     * @param calls_tick a callback calling Sensor::tick()
     */
    void initialize(twizy_wheel_encoder::NodeHandle &nh, int pin,
                    void (*calls_tick)());

    /**
     * Publishes a WheelEncoder
     *
     * @param stamp Time stamp to assign to the stamp field of the published
     * message
     */
    void publish(const ros::Time &stamp);

    /**
     * Callback to be called on encoder ticks
     */
    void tick();
};

} // namespace twizy_wheel_encoder

#endif