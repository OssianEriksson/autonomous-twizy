// This pragma eats program storage space
#pragma GCC optimize("O1")

#include "definitions.h"
#include "sensor.h"

#include <ros.h>

twizy_wheel_encoder::NodeHandle nh;

twizy_wheel_encoder::Parameters params;
twizy_wheel_encoder::Sensor sensor_left("wheel/left/encoder", params,
                                        params.left),
    sensor_right("wheel/right/encoder", params, params.right);

unsigned long last_time;

void setup() {
    nh.initNode();

    // Read ROS parameters
    update_parameters();

    // Inialize wheel encoder sensors
    sensor_left.initialize(nh, 3, tick_sensor_left);
    sensor_right.initialize(nh, 2, tick_sensor_right);
}

/**
 * Read a required ROS parameter by blocking unitl the parameter becomes
 * available
 *
 * @param name Name of parameter
 * @param param Pointer to location of parameter value
 * @param length
 */
template <class T>
void get_required_param(const char *name, T *param, int length = 1) {
    while (!nh.getParam(name, param, length, 1000)) {
        nh.loginfo("Retrying for missing parameter...");
        nh.spinOnce();
    }
}

/**
 * Read ROS parameters
 */
void update_parameters() {
    get_required_param("~covariance", (float *)&params.covariance);

    int cycles_per_revolution;
    float wheel_radius;
    get_required_param("~cycles_per_revolution", (int *)&cycles_per_revolution);
    get_required_param("~wheel_radius", (float *)&wheel_radius);
    params.sensitivity = 1000.0 * TWO_PI * wheel_radius / cycles_per_revolution;

    char *left_frame_id[1] = {params.left.frame_id},
         *right_frame_id[1] = {params.right.frame_id};
    get_required_param("~left_frame_id", (char **)left_frame_id);
    get_required_param("~right_frame_id", (char **)right_frame_id);

    float frequency = 20.0;
    nh.getParam("~frequency", (float *)&frequency);
    params.mspt = 1000.0 / frequency;
}

void loop() {
    unsigned long time = millis();
    if (time - last_time >= params.mspt) {
        // Publish sensor values
        const ros::Time stamp = nh.now();
        sensor_left.publish(stamp);
        sensor_right.publish(stamp);
        nh.spinOnce();

        // If we lost connection through the serial communications channel
        // (which happens e.g. when the ROS node on the other end is terminated)
        // we prepare for re-reading parameters. This is so that when the node
        // on the other end is stopped to update the parameters on the parameter
        // server and then started again the changes will actually be reflected.
        // Remember that the arduino is alwas on and the setup() method will
        // only be called once power is first supplied to the board and not when
        // the ROS node is started.
        if (!nh.connected()) {
            update_parameters();
        }

        last_time = time;
    }
}

void tick_sensor_left() { sensor_left.tick(); }
void tick_sensor_right() { sensor_right.tick(); }