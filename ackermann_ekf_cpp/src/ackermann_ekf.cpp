#include "ackermann_ekf_cpp/ackermann_ekf.h"

#include <Eigen/Dense>
#include <math.h>

namespace ackermann_ekf {
AckermannEkf::AckermannEkf()
    : x_(STATE_SIZE),
      P_(STATE_SIZE, STATE_SIZE),
      Q_(STATE_SIZE, STATE_SIZE),
      I_(STATE_SIZE, STATE_SIZE),
      time_(0.0) {
    Q_.setZero();
    Q_(State::X, State::X) = 1e-3;
    Q_(State::Y, State::Y) = 1e-3;
    Q_(State::Z, State::Z) = 1e-3;
    Q_(State::speed, State::speed) = 1e-2;
    Q_(State::accel, State::accel) = 1e-1;
    Q_(State::Roll, State::Roll) = 1e-2;
    Q_(State::Pitch, State::Pitch) = 1e-2;
    Q_(State::Yaw, State::Yaw) = 1e-2;
    Q_(State::dpitch_dx, State::dpitch_dx) = 1e-1;
    Q_(State::dyaw_dx, State::dyaw_dx) = 1e-1;
    Q_(State::droll_dx, State::droll_dx) = 1e-1;

    P_.setIdentity();
}

void AckermannEkf::process_measurement(const Measurement &measurement) {
    double dt = measurement.time_ - time_;

    if (dt > 0) {
        predict(dt);

        time_ = measurement.time_;
    }

    correct(measurement);
}

void AckermannEkf::predict(double dt) {
    Eigen::VectorXd f(STATE_SIZE);
    Eigen::MatrixXd F(STATE_SIZE, STATE_SIZE);

    // clang-format off

    f(State::X) = x_(State::X) - ((dt * dt) * (x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) + x_(State::dyaw_dx) * x_(State::speed) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll)))) - x_(State::accel) * cos(x_(State::Pitch)) * cos(x_(State::Yaw)))) / 2.0 + dt * x_(State::speed) * cos(x_(State::Pitch)) * cos(x_(State::Yaw));
    f(State::Y) = x_(State::Y) + ((dt * dt) * (x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))) + x_(State::dyaw_dx) * x_(State::speed) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw)))) + x_(State::accel) * cos(x_(State::Pitch)) * sin(x_(State::Yaw)))) / 2.0 + dt * x_(State::speed) * cos(x_(State::Pitch)) * sin(x_(State::Yaw));
    f(State::Z) = x_(State::Z) - ((dt * dt) * (x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) - x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Pitch)) * sin(x_(State::Roll))) + x_(State::accel) * sin(x_(State::Pitch)))) / 2.0 - dt * x_(State::speed) * sin(x_(State::Pitch));
    f(State::speed) = x_(State::speed) + x_(State::accel) * dt;
    f(State::accel) = x_(State::accel);
    f(State::Roll) = (x_(State::Roll) * cos(x_(State::Pitch)) + x_(State::droll_dx) * dt * x_(State::speed) * cos(x_(State::Pitch)) + dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) + x_(State::dpitch_dx) * dt * x_(State::speed) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) / cos(x_(State::Pitch));
    f(State::Pitch) = x_(State::Pitch) + dt * x_(State::speed) * (x_(State::dpitch_dx) * cos(x_(State::Roll)) - x_(State::dyaw_dx) * sin(x_(State::Roll)));
    f(State::Yaw) = (x_(State::Yaw) * cos(x_(State::Pitch)) + dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * dt * x_(State::speed) * sin(x_(State::Roll))) / cos(x_(State::Pitch));
    f(State::droll_dx) = x_(State::droll_dx);
    f(State::dpitch_dx) = x_(State::dpitch_dx);
    f(State::dyaw_dx) = x_(State::dyaw_dx);

    F(State::X, State::X) = 1.0;
    F(State::X, State::speed) = -dt * (-cos(x_(State::Pitch)) * cos(x_(State::Yaw)) + dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Yaw)) + x_(State::dpitch_dx) * dt * x_(State::speed) * sin(x_(State::Roll)) * sin(x_(State::Yaw)) + x_(State::dpitch_dx) * dt * x_(State::speed) * cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch)) - dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll)));
    F(State::X, State::accel) = ((dt * dt) * cos(x_(State::Pitch)) * cos(x_(State::Yaw))) / 2.0;
    F(State::X, State::Roll) = (dt * dt) * x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) - x_(State::dyaw_dx) * x_(State::speed) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch)))) * (-1.0 / 2.0);
    F(State::X, State::Pitch) = dt * cos(x_(State::Yaw)) * (x_(State::speed) * sin(x_(State::Pitch)) * 2.0 + x_(State::accel) * dt * sin(x_(State::Pitch)) + x_(State::dpitch_dx) * dt * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) - dt * x_(State::dyaw_dx) * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * sin(x_(State::Roll))) * (-1.0 / 2.0);
    F(State::X, State::Yaw) = (dt * dt) * (x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))) + x_(State::dyaw_dx) * x_(State::speed) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw)))) + x_(State::accel) * cos(x_(State::Pitch)) * sin(x_(State::Yaw))) * (-1.0 / 2.0) - dt * x_(State::speed) * cos(x_(State::Pitch)) * sin(x_(State::Yaw));
    F(State::X, State::dpitch_dx) = (dt * dt) * (x_(State::speed) * x_(State::speed)) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) * (-1.0 / 2.0);
    F(State::X, State::dyaw_dx) = (dt * dt) * (x_(State::speed) * x_(State::speed)) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) * (-1.0 / 2.0);
    F(State::Y, State::Y) = 1.0;
    F(State::Y, State::speed) = dt * (cos(x_(State::Pitch)) * sin(x_(State::Yaw)) + dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Roll)) * cos(x_(State::Yaw)) + x_(State::dpitch_dx) * dt * x_(State::speed) * cos(x_(State::Yaw)) * sin(x_(State::Roll)) - x_(State::dpitch_dx) * dt * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw)) + dt * x_(State::dyaw_dx) * x_(State::speed) * sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw)));
    F(State::Y, State::accel) = ((dt * dt) * cos(x_(State::Pitch)) * sin(x_(State::Yaw))) / 2.0;
    F(State::Y, State::Roll) = ((dt * dt) * x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw))) - x_(State::dyaw_dx) * x_(State::speed) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))))) / 2.0;
    F(State::Y, State::Pitch) = dt * sin(x_(State::Yaw)) * (x_(State::speed) * sin(x_(State::Pitch)) * 2.0 + x_(State::accel) * dt * sin(x_(State::Pitch)) + x_(State::dpitch_dx) * dt * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) - dt * x_(State::dyaw_dx) * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * sin(x_(State::Roll))) * (-1.0 / 2.0);
    F(State::Y, State::Yaw) = (dt * dt) * (x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) + x_(State::dyaw_dx) * x_(State::speed) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll)))) - x_(State::accel) * cos(x_(State::Pitch)) * cos(x_(State::Yaw))) * (-1.0 / 2.0) + dt * x_(State::speed) * cos(x_(State::Pitch)) * cos(x_(State::Yaw));
    F(State::Y, State::dpitch_dx) = ((dt * dt) * (x_(State::speed) * x_(State::speed)) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw)))) / 2.0;
    F(State::Y, State::dyaw_dx) = ((dt * dt) * (x_(State::speed) * x_(State::speed)) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw)))) / 2.0;
    F(State::Z, State::Z) = 1.0;
    F(State::Z, State::speed) = -dt * (sin(x_(State::Pitch)) + x_(State::dpitch_dx) * dt * x_(State::speed) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) - dt * x_(State::dyaw_dx) * x_(State::speed) * cos(x_(State::Pitch)) * sin(x_(State::Roll)));
    F(State::Z, State::accel) = (dt * dt) * sin(x_(State::Pitch)) * (-1.0 / 2.0);
    F(State::Z, State::Roll) = ((dt * dt) * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * (x_(State::dyaw_dx) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * sin(x_(State::Roll)))) / 2.0;
    F(State::Z, State::Pitch) = (dt * dt) * (x_(State::accel) * cos(x_(State::Pitch)) - x_(State::speed) * (x_(State::dpitch_dx) * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) - x_(State::dyaw_dx) * x_(State::speed) * sin(x_(State::Pitch)) * sin(x_(State::Roll)))) * (-1.0 / 2.0) - dt * x_(State::speed) * cos(x_(State::Pitch));
    F(State::Z, State::dpitch_dx) = (dt * dt) * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) * (-1.0 / 2.0);
    F(State::Z, State::dyaw_dx) = ((dt * dt) * (x_(State::speed) * x_(State::speed)) * cos(x_(State::Pitch)) * sin(x_(State::Roll))) / 2.0;
    F(State::speed, State::speed) = 1.0;
    F(State::speed, State::accel) = dt;
    F(State::accel, State::accel) = 1.0;
    F(State::Roll, State::speed) = (dt * (x_(State::droll_dx) * cos(x_(State::Pitch)) + x_(State::dyaw_dx) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) + x_(State::dpitch_dx) * sin(x_(State::Pitch)) * sin(x_(State::Roll)))) / cos(x_(State::Pitch));
    F(State::Roll, State::Roll) = (cos(x_(State::Pitch)) + x_(State::dpitch_dx) * dt * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) - dt * x_(State::dyaw_dx) * x_(State::speed) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) / cos(x_(State::Pitch));
    F(State::Roll, State::Pitch) = dt * x_(State::speed) * 1.0 / pow(cos(x_(State::Pitch)), 2.0) * (x_(State::dyaw_dx) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * sin(x_(State::Roll)));
    F(State::Roll, State::droll_dx) = dt * x_(State::speed);
    F(State::Roll, State::dpitch_dx) = (dt * x_(State::speed) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) / cos(x_(State::Pitch));
    F(State::Roll, State::dyaw_dx) = (dt * x_(State::speed) * cos(x_(State::Roll)) * sin(x_(State::Pitch))) / cos(x_(State::Pitch));
    F(State::Pitch, State::speed) = dt * (x_(State::dpitch_dx) * cos(x_(State::Roll)) - x_(State::dyaw_dx) * sin(x_(State::Roll)));
    F(State::Pitch, State::Roll) = -dt * x_(State::speed) * (x_(State::dyaw_dx) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * sin(x_(State::Roll)));
    F(State::Pitch, State::Pitch) = 1.0;
    F(State::Pitch, State::dpitch_dx) = dt * x_(State::speed) * cos(x_(State::Roll));
    F(State::Pitch, State::dyaw_dx) = -dt * x_(State::speed) * sin(x_(State::Roll));
    F(State::Yaw, State::speed) = (dt * (x_(State::dyaw_dx) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * sin(x_(State::Roll)))) / cos(x_(State::Pitch));
    F(State::Yaw, State::Roll) = (dt * x_(State::speed) * (x_(State::dpitch_dx) * cos(x_(State::Roll)) - x_(State::dyaw_dx) * sin(x_(State::Roll)))) / cos(x_(State::Pitch));
    F(State::Yaw, State::Pitch) = dt * x_(State::speed) * 1.0 / pow(cos(x_(State::Pitch)), 2.0) * sin(x_(State::Pitch)) * (x_(State::dyaw_dx) * cos(x_(State::Roll)) + x_(State::dpitch_dx) * sin(x_(State::Roll)));
    F(State::Yaw, State::Yaw) = 1.0;
    F(State::Yaw, State::dpitch_dx) = (dt * x_(State::speed) * sin(x_(State::Roll))) / cos(x_(State::Pitch));
    F(State::Yaw, State::dyaw_dx) = (dt * x_(State::speed) * cos(x_(State::Roll))) / cos(x_(State::Pitch));
    F(State::droll_dx, State::droll_dx) = 1.0;
    F(State::dpitch_dx, State::dpitch_dx) = 1.0;
    F(State::dyaw_dx, State::dyaw_dx) = 1.0;

    // clang-format on

    x_ = f;
    P_ = F * P_ * F.transpose();
    P_.noalias() += dt * Q_;
}

void AckermannEkf::correct(const Measurement &measurement) {
    Eigen::VectorXd h(STATE_SIZE);
    Eigen::MatrixXd H(MEASUREMENT_SIZE, STATE_SIZE);

    // clang-format off

    h(Measurement::X) = x_(State::X) - measurement.sensor_position_(1) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) + measurement.sensor_position_(2) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) + measurement.sensor_position_(0) * cos(x_(State::Pitch)) * cos(x_(State::Yaw));
    h(Measurement::Y) = x_(State::Y) + measurement.sensor_position_(1) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw))) - measurement.sensor_position_(2) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))) + measurement.sensor_position_(0) * cos(x_(State::Pitch)) * sin(x_(State::Yaw));
    h(Measurement::Z) = x_(State::Z) - measurement.sensor_position_(0) * sin(x_(State::Pitch)) + measurement.sensor_position_(2) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) + measurement.sensor_position_(1) * cos(x_(State::Pitch)) * sin(x_(State::Roll));
    h(Measurement::dx_dt) = x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(2) - x_(State::dyaw_dx) * measurement.sensor_position_(1) + 1.0);
    h(Measurement::dy_dt) = -x_(State::speed) * (x_(State::droll_dx) * measurement.sensor_position_(2) - x_(State::dyaw_dx) * measurement.sensor_position_(0));
    h(Measurement::dz_dt) = -x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(0) - x_(State::droll_dx) * measurement.sensor_position_(1));
    h(Measurement::d2x_dt2) = x_(State::accel) + x_(State::accel) * x_(State::dpitch_dx) * measurement.sensor_position_(2) - x_(State::accel) * x_(State::dyaw_dx) * measurement.sensor_position_(1) - x_(State::dpitch_dx) * x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(0) * x_(State::speed) - x_(State::droll_dx) * measurement.sensor_position_(1) * x_(State::speed)) + x_(State::dyaw_dx) * x_(State::speed) * (x_(State::droll_dx) * measurement.sensor_position_(2) * x_(State::speed) - x_(State::dyaw_dx) * measurement.sensor_position_(0) * x_(State::speed));
    h(Measurement::d2y_dt2) = x_(State::dyaw_dx) * (x_(State::speed) * x_(State::speed)) * 2.0 - x_(State::accel) * x_(State::droll_dx) * measurement.sensor_position_(2) + x_(State::accel) * x_(State::dyaw_dx) * measurement.sensor_position_(0) + x_(State::droll_dx) * x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(0) * x_(State::speed) - x_(State::droll_dx) * measurement.sensor_position_(1) * x_(State::speed)) + x_(State::dyaw_dx) * x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(2) * x_(State::speed) - x_(State::dyaw_dx) * measurement.sensor_position_(1) * x_(State::speed));
    h(Measurement::d2z_dt2) = x_(State::dpitch_dx) * (x_(State::speed) * x_(State::speed)) * -2.0 - x_(State::accel) * x_(State::dpitch_dx) * measurement.sensor_position_(0) + x_(State::accel) * x_(State::droll_dx) * measurement.sensor_position_(1) - x_(State::dpitch_dx) * x_(State::speed) * (x_(State::dpitch_dx) * measurement.sensor_position_(2) * x_(State::speed) - x_(State::dyaw_dx) * measurement.sensor_position_(1) * x_(State::speed)) - x_(State::droll_dx) * x_(State::speed) * (x_(State::droll_dx) * measurement.sensor_position_(2) * x_(State::speed) - x_(State::dyaw_dx) * measurement.sensor_position_(0) * x_(State::speed));
    h(Measurement::Roll) = x_(State::Roll);
    h(Measurement::Pitch) = x_(State::Pitch);
    h(Measurement::Yaw) = x_(State::Yaw);
    h(Measurement::droll_dt) = x_(State::droll_dx) * x_(State::speed);
    h(Measurement::dpitch_dt) = x_(State::dpitch_dx) * x_(State::speed);
    h(Measurement::dyaw_dt) = x_(State::dyaw_dx) * x_(State::speed);

    H(Measurement::X, State::X) = 1.0;
    H(Measurement::X, State::Roll) = measurement.sensor_position_(1) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) + measurement.sensor_position_(2) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll)));
    H(Measurement::X, State::Pitch) = cos(x_(State::Yaw)) * (-measurement.sensor_position_(0) * sin(x_(State::Pitch)) + measurement.sensor_position_(2) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) + measurement.sensor_position_(1) * cos(x_(State::Pitch)) * sin(x_(State::Roll)));
    H(Measurement::X, State::Yaw) = -measurement.sensor_position_(1) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw))) + measurement.sensor_position_(2) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))) - measurement.sensor_position_(0) * cos(x_(State::Pitch)) * sin(x_(State::Yaw));
    H(Measurement::Y, State::Y) = 1.0;
    H(Measurement::Y, State::Roll) = -measurement.sensor_position_(1) * (cos(x_(State::Yaw)) * sin(x_(State::Roll)) - cos(x_(State::Roll)) * sin(x_(State::Pitch)) * sin(x_(State::Yaw))) - measurement.sensor_position_(2) * (cos(x_(State::Roll)) * cos(x_(State::Yaw)) + sin(x_(State::Pitch)) * sin(x_(State::Roll)) * sin(x_(State::Yaw)));
    H(Measurement::Y, State::Pitch) = sin(x_(State::Yaw)) * (-measurement.sensor_position_(0) * sin(x_(State::Pitch)) + measurement.sensor_position_(2) * cos(x_(State::Pitch)) * cos(x_(State::Roll)) + measurement.sensor_position_(1) * cos(x_(State::Pitch)) * sin(x_(State::Roll)));
    H(Measurement::Y, State::Yaw) = -measurement.sensor_position_(1) * (cos(x_(State::Roll)) * sin(x_(State::Yaw)) - cos(x_(State::Yaw)) * sin(x_(State::Pitch)) * sin(x_(State::Roll))) + measurement.sensor_position_(2) * (sin(x_(State::Roll)) * sin(x_(State::Yaw)) + cos(x_(State::Roll)) * cos(x_(State::Yaw)) * sin(x_(State::Pitch))) + measurement.sensor_position_(0) * cos(x_(State::Pitch)) * cos(x_(State::Yaw));
    H(Measurement::Z, State::Z) = 1.0;
    H(Measurement::Z, State::Roll) = cos(x_(State::Pitch)) * (measurement.sensor_position_(1) * cos(x_(State::Roll)) - measurement.sensor_position_(2) * sin(x_(State::Roll)));
    H(Measurement::Z, State::Pitch) = -measurement.sensor_position_(0) * cos(x_(State::Pitch)) - measurement.sensor_position_(2) * cos(x_(State::Roll)) * sin(x_(State::Pitch)) - measurement.sensor_position_(1) * sin(x_(State::Pitch)) * sin(x_(State::Roll));
    H(Measurement::dx_dt, State::speed) = x_(State::dpitch_dx) * measurement.sensor_position_(2) - x_(State::dyaw_dx) * measurement.sensor_position_(1) + 1.0;
    H(Measurement::dx_dt, State::dpitch_dx) = measurement.sensor_position_(2) * x_(State::speed);
    H(Measurement::dx_dt, State::dyaw_dx) = -measurement.sensor_position_(1) * x_(State::speed);
    H(Measurement::dy_dt, State::speed) = -x_(State::droll_dx) * measurement.sensor_position_(2) + x_(State::dyaw_dx) * measurement.sensor_position_(0);
    H(Measurement::dy_dt, State::droll_dx) = -measurement.sensor_position_(2) * x_(State::speed);
    H(Measurement::dy_dt, State::dyaw_dx) = measurement.sensor_position_(0) * x_(State::speed);
    H(Measurement::dz_dt, State::speed) = -x_(State::dpitch_dx) * measurement.sensor_position_(0) + x_(State::droll_dx) * measurement.sensor_position_(1);
    H(Measurement::dz_dt, State::droll_dx) = measurement.sensor_position_(1) * x_(State::speed);
    H(Measurement::dz_dt, State::dpitch_dx) = -measurement.sensor_position_(0) * x_(State::speed);
    H(Measurement::d2x_dt2, State::speed) = x_(State::speed) * ((x_(State::dpitch_dx) * x_(State::dpitch_dx)) * measurement.sensor_position_(0) + (x_(State::dyaw_dx) * x_(State::dyaw_dx)) * measurement.sensor_position_(0) - x_(State::dpitch_dx) * x_(State::droll_dx) * measurement.sensor_position_(1) - x_(State::droll_dx) * x_(State::dyaw_dx) * measurement.sensor_position_(2)) * -2.0;
    H(Measurement::d2x_dt2, State::accel) = x_(State::dpitch_dx) * measurement.sensor_position_(2) - x_(State::dyaw_dx) * measurement.sensor_position_(1) + 1.0;
    H(Measurement::d2x_dt2, State::droll_dx) = (x_(State::speed) * x_(State::speed)) * (x_(State::dpitch_dx) * measurement.sensor_position_(1) + x_(State::dyaw_dx) * measurement.sensor_position_(2));
    H(Measurement::d2x_dt2, State::dpitch_dx) = x_(State::accel) * measurement.sensor_position_(2) - x_(State::dpitch_dx) * measurement.sensor_position_(0) * (x_(State::speed) * x_(State::speed)) * 2.0 + x_(State::droll_dx) * measurement.sensor_position_(1) * (x_(State::speed) * x_(State::speed));
    H(Measurement::d2x_dt2, State::dyaw_dx) = -x_(State::accel) * measurement.sensor_position_(1) + x_(State::droll_dx) * measurement.sensor_position_(2) * (x_(State::speed) * x_(State::speed)) - x_(State::dyaw_dx) * measurement.sensor_position_(0) * (x_(State::speed) * x_(State::speed)) * 2.0;
    H(Measurement::d2y_dt2, State::speed) = x_(State::speed) * (x_(State::dyaw_dx) * 2.0 - (x_(State::droll_dx) * x_(State::droll_dx)) * measurement.sensor_position_(1) - (x_(State::dyaw_dx) * x_(State::dyaw_dx)) * measurement.sensor_position_(1) + x_(State::dpitch_dx) * x_(State::droll_dx) * measurement.sensor_position_(0) + x_(State::dpitch_dx) * x_(State::dyaw_dx) * measurement.sensor_position_(2)) * 2.0;
    H(Measurement::d2y_dt2, State::accel) = -x_(State::droll_dx) * measurement.sensor_position_(2) + x_(State::dyaw_dx) * measurement.sensor_position_(0);
    H(Measurement::d2y_dt2, State::droll_dx) = -x_(State::accel) * measurement.sensor_position_(2) + x_(State::dpitch_dx) * measurement.sensor_position_(0) * (x_(State::speed) * x_(State::speed)) - x_(State::droll_dx) * measurement.sensor_position_(1) * (x_(State::speed) * x_(State::speed)) * 2.0;
    H(Measurement::d2y_dt2, State::dpitch_dx) = (x_(State::speed) * x_(State::speed)) * (x_(State::droll_dx) * measurement.sensor_position_(0) + x_(State::dyaw_dx) * measurement.sensor_position_(2));
    H(Measurement::d2y_dt2, State::dyaw_dx) = x_(State::accel) * measurement.sensor_position_(0) + (x_(State::speed) * x_(State::speed)) * 2.0 + x_(State::dpitch_dx) * measurement.sensor_position_(2) * (x_(State::speed) * x_(State::speed)) - x_(State::dyaw_dx) * measurement.sensor_position_(1) * (x_(State::speed) * x_(State::speed)) * 2.0;
    H(Measurement::d2z_dt2, State::speed) = x_(State::speed) * (x_(State::dpitch_dx) * 2.0 + (x_(State::dpitch_dx) * x_(State::dpitch_dx)) * measurement.sensor_position_(2) + (x_(State::droll_dx) * x_(State::droll_dx)) * measurement.sensor_position_(2) - x_(State::dpitch_dx) * x_(State::dyaw_dx) * measurement.sensor_position_(1) - x_(State::droll_dx) * x_(State::dyaw_dx) * measurement.sensor_position_(0)) * -2.0;
    H(Measurement::d2z_dt2, State::accel) = -x_(State::dpitch_dx) * measurement.sensor_position_(0) + x_(State::droll_dx) * measurement.sensor_position_(1);
    H(Measurement::d2z_dt2, State::droll_dx) = x_(State::accel) * measurement.sensor_position_(1) - x_(State::droll_dx) * measurement.sensor_position_(2) * (x_(State::speed) * x_(State::speed)) * 2.0 + x_(State::dyaw_dx) * measurement.sensor_position_(0) * (x_(State::speed) * x_(State::speed));
    H(Measurement::d2z_dt2, State::dpitch_dx) = -x_(State::accel) * measurement.sensor_position_(0) - (x_(State::speed) * x_(State::speed)) * 2.0 - x_(State::dpitch_dx) * measurement.sensor_position_(2) * (x_(State::speed) * x_(State::speed)) * 2.0 + x_(State::dyaw_dx) * measurement.sensor_position_(1) * (x_(State::speed) * x_(State::speed));
    H(Measurement::d2z_dt2, State::dyaw_dx) = (x_(State::speed) * x_(State::speed)) * (x_(State::dpitch_dx) * measurement.sensor_position_(1) + x_(State::droll_dx) * measurement.sensor_position_(0));
    H(Measurement::Roll, State::Roll) = 1.0;
    H(Measurement::Pitch, State::Pitch) = 1.0;
    H(Measurement::Yaw, State::Yaw) = 1.0;
    H(Measurement::droll_dt, State::speed) = x_(State::droll_dx);
    H(Measurement::droll_dt, State::droll_dx) = x_(State::speed);
    H(Measurement::dpitch_dt, State::speed) = x_(State::dpitch_dx);
    H(Measurement::dpitch_dt, State::dpitch_dx) = x_(State::speed);
    H(Measurement::dyaw_dt, State::speed) = x_(State::dyaw_dx);
    H(Measurement::dyaw_dt, State::dyaw_dx) = x_(State::speed);

    // clang-format on

    Eigen::MatrixXd R = measurement.R_;
    Eigen::VectorXd y(MEASUREMENT_SIZE);
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        if (R(i, i) <= 0) {
            y(i) = 0.0;

            R.row(i).setConstant(0.0);
            R.col(i).setConstant(0.0);
            R(i, i) = INF_COVARIANCE;
        } else {
            y(i) = measurement.z_(i) - h(i);

            if (R(i, i) < MIN_COVARIANCE) {
                R(i, i) = MIN_COVARIANCE;
            }
        }
    }

    Eigen::MatrixXd K =
        P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    x_.noalias() += K * y;
    P_ = (I_ - K * H) * P_;
}
} // namespace ackermann_ekf