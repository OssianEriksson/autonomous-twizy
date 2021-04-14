#ifndef TWIZY_WHEEL_ENCODER_SENSOR
#define TWIZY_WHEEL_ENCODER_SENSOR

#include "definitions.h"

#include <twizy_wheel_encoder/WheelEncoder.h>

namespace twizy_wheel_encoder {

class Sensor {
  private:
    WheelEncoder msg_;

    const Parameters &params_;
    const Parameters::Side &side_;

    unsigned long last_time_;

  public:
    ros::Publisher publisher;

    Sensor(const char *topic_name, const Parameters &params,
           const Parameters::Side &side);

    void initialize(twizy_wheel_encoder::NodeHandle &nh, int pin,
                    void (*calls_tick)());
    void publish(const ros::Time &stamp);
    void tick();
};

} // namespace twizy_wheel_encoder

#endif