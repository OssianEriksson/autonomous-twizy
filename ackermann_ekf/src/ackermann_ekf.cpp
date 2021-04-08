#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/imu_sensor.h"
#include "ackermann_ekf/navsatfix_sensor.h"
#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/sensor_array.h"

namespace ackermann_ekf {
AckermannEkf::AckermannEkf(const Eigen::VectorXd x_min,
                           const Eigen::VectorXd x_max, double wheelbase,
                           double control_acceleration_gain,
                           double max_control_acceleration,
                           double control_angle_speed_gain,
                           double max_control_angle_speed)
    : x(STATE_SIZE),
      P(STATE_SIZE, STATE_SIZE),
      Q(STATE_SIZE, STATE_SIZE),
      x_min_(x_min),
      x_max_(x_max),
      wheelbase_(wheelbase),
      control_acceleration_gain_(control_acceleration_gain),
      max_control_acceleration_(max_control_acceleration),
      control_angle_speed_gain_(control_angle_speed_gain),
      max_control_angle_speed_(max_control_angle_speed) {
    x.setZero();
    P.setIdentity();

    Q.setZero();
    Q(State::X, State::X) = 1e-4;
    Q(State::Y, State::Y) = 1e-4;
    Q(State::Z, State::Z) = 1e-4;
    Q(State::speed, State::speed) = 1e-2;
    Q(State::accel, State::accel) = 1e-0;
    Q(State::Roll, State::Roll) = 1e-3;
    Q(State::Pitch, State::Pitch) = 1e-3;
    Q(State::Yaw, State::Yaw) = 1e-2;
    Q(State::droll_dx, State::droll_dx) = 1e-1;
    Q(State::dpitch_dx, State::dpitch_dx) = 1e-1;
    Q(State::dyaw_dx, State::dyaw_dx) = 1e-0;
}

void AckermannEkf::process_measurement(const Measurement &measurement) {
    bring_time_forward_to(measurement.time);

    correct(measurement);
}

void AckermannEkf::process_control_signal(const ControlSignal &control_signal) {
    bring_time_forward_to(control_signal.time);

    control_signal_enabled_ = true;
    control_signal_ = control_signal;
}

void AckermannEkf::bring_time_forward_to(double time) {
    if (time > this->time) {
        if (this->time >= 0) {
            predict(time - this->time);
        }
        this->time = time;
    }
}

void AckermannEkf::predict(double dt) {
    if (control_signal_enabled_) {
        double acceleration = boost::algorithm::clamp(
            (control_signal_.u(ControlSignal::speed) - x(State::speed)) *
                control_acceleration_gain_,
            -max_control_acceleration_, max_control_acceleration_);

        double jerk_dt = acceleration - x(State::accel);
        x(State::accel) = acceleration;
        x(State::speed) += jerk_dt * dt / 2;

        double steering_angle = atan(wheelbase_ * x(State::dyaw_dx));
        steering_angle +=
            boost::algorithm::clamp(
                (control_signal_.u(ControlSignal::angle) - steering_angle) *
                    control_angle_speed_gain_,
                -max_control_angle_speed_, max_control_angle_speed_) *
            dt;
        steering_angle = boost::algorithm::clamp(steering_angle, -M_PI * 0.499,
                                                 M_PI * 0.499);

        x(State::dyaw_dx) = tan(steering_angle) / wheelbase_;

        constrain_state();
    }

    Eigen::VectorXd f(STATE_SIZE);
    Eigen::MatrixXd F(STATE_SIZE, STATE_SIZE);
    f.setZero();
    F.setZero();

    // clang-format off

    f(State::X) = x(State::X)-((dt*dt)*(x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))+x(State::dyaw_dx)*x(State::speed)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll))))-x(State::accel)*cos(x(State::Pitch))*cos(x(State::Yaw))))/2.0+dt*x(State::speed)*cos(x(State::Pitch))*cos(x(State::Yaw));
    f(State::Y) = x(State::Y)+((dt*dt)*(x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))+x(State::dyaw_dx)*x(State::speed)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw))))+x(State::accel)*cos(x(State::Pitch))*sin(x(State::Yaw))))/2.0+dt*x(State::speed)*cos(x(State::Pitch))*sin(x(State::Yaw));
    f(State::Z) = x(State::Z)-((dt*dt)*(x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*cos(x(State::Pitch))*cos(x(State::Roll))-x(State::dyaw_dx)*x(State::speed)*cos(x(State::Pitch))*sin(x(State::Roll)))+x(State::accel)*sin(x(State::Pitch))))/2.0-dt*x(State::speed)*sin(x(State::Pitch));
    f(State::speed) = x(State::speed)+x(State::accel)*dt;
    f(State::accel) = x(State::accel);
    f(State::Roll) = (x(State::Roll)*cos(x(State::Pitch))+x(State::droll_dx)*dt*x(State::speed)*cos(x(State::Pitch))+dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Roll))*sin(x(State::Pitch))+x(State::dpitch_dx)*dt*x(State::speed)*sin(x(State::Pitch))*sin(x(State::Roll)))/cos(x(State::Pitch));
    f(State::Pitch) = x(State::Pitch)+dt*x(State::speed)*(x(State::dpitch_dx)*cos(x(State::Roll))-x(State::dyaw_dx)*sin(x(State::Roll)));
    f(State::Yaw) = (x(State::Yaw)*cos(x(State::Pitch))+dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Roll))+x(State::dpitch_dx)*dt*x(State::speed)*sin(x(State::Roll)))/cos(x(State::Pitch));
    f(State::droll_dx) = x(State::droll_dx);
    f(State::dpitch_dx) = x(State::dpitch_dx);
    f(State::dyaw_dx) = x(State::dyaw_dx);

    F(State::X, State::X) = 1.0;
    F(State::X, State::speed) = -dt*(-cos(x(State::Pitch))*cos(x(State::Yaw))+dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Roll))*sin(x(State::Yaw))+x(State::dpitch_dx)*dt*x(State::speed)*sin(x(State::Roll))*sin(x(State::Yaw))+x(State::dpitch_dx)*dt*x(State::speed)*cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch))-dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)));
    F(State::X, State::accel) = ((dt*dt)*cos(x(State::Pitch))*cos(x(State::Yaw)))/2.0;
    F(State::X, State::Roll) = (dt*dt)*x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)))-x(State::dyaw_dx)*x(State::speed)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch))))*(-1.0/2.0);
    F(State::X, State::Pitch) = dt*cos(x(State::Yaw))*(x(State::speed)*sin(x(State::Pitch))*2.0+x(State::accel)*dt*sin(x(State::Pitch))+x(State::dpitch_dx)*dt*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*cos(x(State::Roll))-dt*x(State::dyaw_dx)*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*sin(x(State::Roll)))*(-1.0/2.0);
    F(State::X, State::Yaw) = (dt*dt)*(x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))+x(State::dyaw_dx)*x(State::speed)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw))))+x(State::accel)*cos(x(State::Pitch))*sin(x(State::Yaw)))*(-1.0/2.0)-dt*x(State::speed)*cos(x(State::Pitch))*sin(x(State::Yaw));
    F(State::X, State::dpitch_dx) = (dt*dt)*(x(State::speed)*x(State::speed))*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))*(-1.0/2.0);
    F(State::X, State::dyaw_dx) = (dt*dt)*(x(State::speed)*x(State::speed))*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)))*(-1.0/2.0);
    F(State::Y, State::Y) = 1.0;
    F(State::Y, State::speed) = dt*(cos(x(State::Pitch))*sin(x(State::Yaw))+dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Roll))*cos(x(State::Yaw))+x(State::dpitch_dx)*dt*x(State::speed)*cos(x(State::Yaw))*sin(x(State::Roll))-x(State::dpitch_dx)*dt*x(State::speed)*cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw))+dt*x(State::dyaw_dx)*x(State::speed)*sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw)));
    F(State::Y, State::accel) = ((dt*dt)*cos(x(State::Pitch))*sin(x(State::Yaw)))/2.0;
    F(State::Y, State::Roll) = ((dt*dt)*x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw)))-x(State::dyaw_dx)*x(State::speed)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))))/2.0;
    F(State::Y, State::Pitch) = dt*sin(x(State::Yaw))*(x(State::speed)*sin(x(State::Pitch))*2.0+x(State::accel)*dt*sin(x(State::Pitch))+x(State::dpitch_dx)*dt*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*cos(x(State::Roll))-dt*x(State::dyaw_dx)*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*sin(x(State::Roll)))*(-1.0/2.0);
    F(State::Y, State::Yaw) = (dt*dt)*(x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))+x(State::dyaw_dx)*x(State::speed)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll))))-x(State::accel)*cos(x(State::Pitch))*cos(x(State::Yaw)))*(-1.0/2.0)+dt*x(State::speed)*cos(x(State::Pitch))*cos(x(State::Yaw));
    F(State::Y, State::dpitch_dx) = ((dt*dt)*(x(State::speed)*x(State::speed))*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw))))/2.0;
    F(State::Y, State::dyaw_dx) = ((dt*dt)*(x(State::speed)*x(State::speed))*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw))))/2.0;
    F(State::Z, State::Z) = 1.0;
    F(State::Z, State::speed) = -dt*(sin(x(State::Pitch))+x(State::dpitch_dx)*dt*x(State::speed)*cos(x(State::Pitch))*cos(x(State::Roll))-dt*x(State::dyaw_dx)*x(State::speed)*cos(x(State::Pitch))*sin(x(State::Roll)));
    F(State::Z, State::accel) = (dt*dt)*sin(x(State::Pitch))*(-1.0/2.0);
    F(State::Z, State::Roll) = ((dt*dt)*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*(x(State::dyaw_dx)*cos(x(State::Roll))+x(State::dpitch_dx)*sin(x(State::Roll))))/2.0;
    F(State::Z, State::Pitch) = (dt*dt)*(x(State::accel)*cos(x(State::Pitch))-x(State::speed)*(x(State::dpitch_dx)*x(State::speed)*cos(x(State::Roll))*sin(x(State::Pitch))-x(State::dyaw_dx)*x(State::speed)*sin(x(State::Pitch))*sin(x(State::Roll))))*(-1.0/2.0)-dt*x(State::speed)*cos(x(State::Pitch));
    F(State::Z, State::dpitch_dx) = (dt*dt)*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*cos(x(State::Roll))*(-1.0/2.0);
    F(State::Z, State::dyaw_dx) = ((dt*dt)*(x(State::speed)*x(State::speed))*cos(x(State::Pitch))*sin(x(State::Roll)))/2.0;
    F(State::speed, State::speed) = 1.0;
    F(State::speed, State::accel) = dt;
    F(State::accel, State::accel) = 1.0;
    F(State::Roll, State::speed) = (dt*(x(State::droll_dx)*cos(x(State::Pitch))+x(State::dyaw_dx)*cos(x(State::Roll))*sin(x(State::Pitch))+x(State::dpitch_dx)*sin(x(State::Pitch))*sin(x(State::Roll))))/cos(x(State::Pitch));
    F(State::Roll, State::Roll) = (cos(x(State::Pitch))+x(State::dpitch_dx)*dt*x(State::speed)*cos(x(State::Roll))*sin(x(State::Pitch))-dt*x(State::dyaw_dx)*x(State::speed)*sin(x(State::Pitch))*sin(x(State::Roll)))/cos(x(State::Pitch));
    F(State::Roll, State::Pitch) = dt*x(State::speed)*1.0/pow(cos(x(State::Pitch)),2.0)*(x(State::dyaw_dx)*cos(x(State::Roll))+x(State::dpitch_dx)*sin(x(State::Roll)));
    F(State::Roll, State::droll_dx) = dt*x(State::speed);
    F(State::Roll, State::dpitch_dx) = (dt*x(State::speed)*sin(x(State::Pitch))*sin(x(State::Roll)))/cos(x(State::Pitch));
    F(State::Roll, State::dyaw_dx) = (dt*x(State::speed)*cos(x(State::Roll))*sin(x(State::Pitch)))/cos(x(State::Pitch));
    F(State::Pitch, State::speed) = dt*(x(State::dpitch_dx)*cos(x(State::Roll))-x(State::dyaw_dx)*sin(x(State::Roll)));
    F(State::Pitch, State::Roll) = -dt*x(State::speed)*(x(State::dyaw_dx)*cos(x(State::Roll))+x(State::dpitch_dx)*sin(x(State::Roll)));
    F(State::Pitch, State::Pitch) = 1.0;
    F(State::Pitch, State::dpitch_dx) = dt*x(State::speed)*cos(x(State::Roll));
    F(State::Pitch, State::dyaw_dx) = -dt*x(State::speed)*sin(x(State::Roll));
    F(State::Yaw, State::speed) = (dt*(x(State::dyaw_dx)*cos(x(State::Roll))+x(State::dpitch_dx)*sin(x(State::Roll))))/cos(x(State::Pitch));
    F(State::Yaw, State::Roll) = (dt*x(State::speed)*(x(State::dpitch_dx)*cos(x(State::Roll))-x(State::dyaw_dx)*sin(x(State::Roll))))/cos(x(State::Pitch));
    F(State::Yaw, State::Pitch) = dt*x(State::speed)*1.0/pow(cos(x(State::Pitch)),2.0)*sin(x(State::Pitch))*(x(State::dyaw_dx)*cos(x(State::Roll))+x(State::dpitch_dx)*sin(x(State::Roll)));
    F(State::Yaw, State::Yaw) = 1.0;
    F(State::Yaw, State::dpitch_dx) = (dt*x(State::speed)*sin(x(State::Roll)))/cos(x(State::Pitch));
    F(State::Yaw, State::dyaw_dx) = (dt*x(State::speed)*cos(x(State::Roll)))/cos(x(State::Pitch));
    F(State::droll_dx, State::droll_dx) = 1.0;
    F(State::dpitch_dx, State::dpitch_dx) = 1.0;
    F(State::dyaw_dx, State::dyaw_dx) = 1.0;

    // clang-format on

    x = f;
    constrain_state();
    P = F * P * F.transpose() + dt * Q;
}

void AckermannEkf::correct(const Measurement &measurement) {
    Eigen::VectorXd h(MEASUREMENT_SIZE);
    Eigen::MatrixXd H(MEASUREMENT_SIZE, STATE_SIZE);
    h.setZero();
    H.setZero();

    // clang-format off

    h(Measurement::X) = x(State::X)-measurement.sensor_position(1)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)))+measurement.sensor_position(2)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))+measurement.sensor_position(0)*cos(x(State::Pitch))*cos(x(State::Yaw));
    h(Measurement::Y) = x(State::Y)+measurement.sensor_position(1)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw)))-measurement.sensor_position(2)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))+measurement.sensor_position(0)*cos(x(State::Pitch))*sin(x(State::Yaw));
    h(Measurement::Z) = x(State::Z)-measurement.sensor_position(0)*sin(x(State::Pitch))+measurement.sensor_position(2)*cos(x(State::Pitch))*cos(x(State::Roll))+measurement.sensor_position(1)*cos(x(State::Pitch))*sin(x(State::Roll));
    h(Measurement::dx_dt) = x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(2)-x(State::dyaw_dx)*measurement.sensor_position(1)+1.0);
    h(Measurement::dy_dt) = -x(State::speed)*(x(State::droll_dx)*measurement.sensor_position(2)-x(State::dyaw_dx)*measurement.sensor_position(0));
    h(Measurement::dz_dt) = -x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(0)-x(State::droll_dx)*measurement.sensor_position(1));
    h(Measurement::d2x_dt2) = x(State::accel)-measurement.gravity*sin(x(State::Pitch))+x(State::accel)*x(State::dpitch_dx)*measurement.sensor_position(2)-x(State::accel)*x(State::dyaw_dx)*measurement.sensor_position(1)-x(State::dpitch_dx)*x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(0)*x(State::speed)-x(State::droll_dx)*measurement.sensor_position(1)*x(State::speed))+x(State::dyaw_dx)*x(State::speed)*(x(State::droll_dx)*measurement.sensor_position(2)*x(State::speed)-x(State::dyaw_dx)*measurement.sensor_position(0)*x(State::speed));
    h(Measurement::d2y_dt2) = x(State::dyaw_dx)*(x(State::speed)*x(State::speed))*2.0+measurement.gravity*cos(x(State::Pitch))*sin(x(State::Roll))-x(State::accel)*x(State::droll_dx)*measurement.sensor_position(2)+x(State::accel)*x(State::dyaw_dx)*measurement.sensor_position(0)+x(State::droll_dx)*x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(0)*x(State::speed)-x(State::droll_dx)*measurement.sensor_position(1)*x(State::speed))+x(State::dyaw_dx)*x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(2)*x(State::speed)-x(State::dyaw_dx)*measurement.sensor_position(1)*x(State::speed));
    h(Measurement::d2z_dt2) = x(State::dpitch_dx)*(x(State::speed)*x(State::speed))*-2.0+measurement.gravity*cos(x(State::Pitch))*cos(x(State::Roll))-x(State::accel)*x(State::dpitch_dx)*measurement.sensor_position(0)+x(State::accel)*x(State::droll_dx)*measurement.sensor_position(1)-x(State::dpitch_dx)*x(State::speed)*(x(State::dpitch_dx)*measurement.sensor_position(2)*x(State::speed)-x(State::dyaw_dx)*measurement.sensor_position(1)*x(State::speed))-x(State::droll_dx)*x(State::speed)*(x(State::droll_dx)*measurement.sensor_position(2)*x(State::speed)-x(State::dyaw_dx)*measurement.sensor_position(0)*x(State::speed));
    h(Measurement::Roll) = x(State::Roll);
    h(Measurement::Pitch) = x(State::Pitch);
    h(Measurement::Yaw) = x(State::Yaw);
    h(Measurement::droll_dt) = x(State::droll_dx)*x(State::speed);
    h(Measurement::dpitch_dt) = x(State::dpitch_dx)*x(State::speed);
    h(Measurement::dyaw_dt) = x(State::dyaw_dx)*x(State::speed);

    H(Measurement::X, State::X) = 1.0;
    H(Measurement::X, State::Roll) = measurement.sensor_position(1)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))+measurement.sensor_position(2)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)));
    H(Measurement::X, State::Pitch) = cos(x(State::Yaw))*(-measurement.sensor_position(0)*sin(x(State::Pitch))+measurement.sensor_position(2)*cos(x(State::Pitch))*cos(x(State::Roll))+measurement.sensor_position(1)*cos(x(State::Pitch))*sin(x(State::Roll)));
    H(Measurement::X, State::Yaw) = -measurement.sensor_position(1)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw)))+measurement.sensor_position(2)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))-measurement.sensor_position(0)*cos(x(State::Pitch))*sin(x(State::Yaw));
    H(Measurement::Y, State::Y) = 1.0;
    H(Measurement::Y, State::Roll) = -measurement.sensor_position(1)*(cos(x(State::Yaw))*sin(x(State::Roll))-cos(x(State::Roll))*sin(x(State::Pitch))*sin(x(State::Yaw)))-measurement.sensor_position(2)*(cos(x(State::Roll))*cos(x(State::Yaw))+sin(x(State::Pitch))*sin(x(State::Roll))*sin(x(State::Yaw)));
    H(Measurement::Y, State::Pitch) = sin(x(State::Yaw))*(-measurement.sensor_position(0)*sin(x(State::Pitch))+measurement.sensor_position(2)*cos(x(State::Pitch))*cos(x(State::Roll))+measurement.sensor_position(1)*cos(x(State::Pitch))*sin(x(State::Roll)));
    H(Measurement::Y, State::Yaw) = -measurement.sensor_position(1)*(cos(x(State::Roll))*sin(x(State::Yaw))-cos(x(State::Yaw))*sin(x(State::Pitch))*sin(x(State::Roll)))+measurement.sensor_position(2)*(sin(x(State::Roll))*sin(x(State::Yaw))+cos(x(State::Roll))*cos(x(State::Yaw))*sin(x(State::Pitch)))+measurement.sensor_position(0)*cos(x(State::Pitch))*cos(x(State::Yaw));
    H(Measurement::Z, State::Z) = 1.0;
    H(Measurement::Z, State::Roll) = cos(x(State::Pitch))*(measurement.sensor_position(1)*cos(x(State::Roll))-measurement.sensor_position(2)*sin(x(State::Roll)));
    H(Measurement::Z, State::Pitch) = -measurement.sensor_position(0)*cos(x(State::Pitch))-measurement.sensor_position(2)*cos(x(State::Roll))*sin(x(State::Pitch))-measurement.sensor_position(1)*sin(x(State::Pitch))*sin(x(State::Roll));
    H(Measurement::dx_dt, State::speed) = x(State::dpitch_dx)*measurement.sensor_position(2)-x(State::dyaw_dx)*measurement.sensor_position(1)+1.0;
    H(Measurement::dx_dt, State::dpitch_dx) = measurement.sensor_position(2)*x(State::speed);
    H(Measurement::dx_dt, State::dyaw_dx) = -measurement.sensor_position(1)*x(State::speed);
    H(Measurement::dy_dt, State::speed) = -x(State::droll_dx)*measurement.sensor_position(2)+x(State::dyaw_dx)*measurement.sensor_position(0);
    H(Measurement::dy_dt, State::droll_dx) = -measurement.sensor_position(2)*x(State::speed);
    H(Measurement::dy_dt, State::dyaw_dx) = measurement.sensor_position(0)*x(State::speed);
    H(Measurement::dz_dt, State::speed) = -x(State::dpitch_dx)*measurement.sensor_position(0)+x(State::droll_dx)*measurement.sensor_position(1);
    H(Measurement::dz_dt, State::droll_dx) = measurement.sensor_position(1)*x(State::speed);
    H(Measurement::dz_dt, State::dpitch_dx) = -measurement.sensor_position(0)*x(State::speed);
    H(Measurement::d2x_dt2, State::speed) = x(State::speed)*((x(State::dpitch_dx)*x(State::dpitch_dx))*measurement.sensor_position(0)+(x(State::dyaw_dx)*x(State::dyaw_dx))*measurement.sensor_position(0)-x(State::dpitch_dx)*x(State::droll_dx)*measurement.sensor_position(1)-x(State::droll_dx)*x(State::dyaw_dx)*measurement.sensor_position(2))*-2.0;
    H(Measurement::d2x_dt2, State::accel) = x(State::dpitch_dx)*measurement.sensor_position(2)-x(State::dyaw_dx)*measurement.sensor_position(1)+1.0;
    H(Measurement::d2x_dt2, State::Pitch) = -measurement.gravity*cos(x(State::Pitch));
    H(Measurement::d2x_dt2, State::droll_dx) = (x(State::speed)*x(State::speed))*(x(State::dpitch_dx)*measurement.sensor_position(1)+x(State::dyaw_dx)*measurement.sensor_position(2));
    H(Measurement::d2x_dt2, State::dpitch_dx) = x(State::accel)*measurement.sensor_position(2)-x(State::dpitch_dx)*measurement.sensor_position(0)*(x(State::speed)*x(State::speed))*2.0+x(State::droll_dx)*measurement.sensor_position(1)*(x(State::speed)*x(State::speed));
    H(Measurement::d2x_dt2, State::dyaw_dx) = -x(State::accel)*measurement.sensor_position(1)+x(State::droll_dx)*measurement.sensor_position(2)*(x(State::speed)*x(State::speed))-x(State::dyaw_dx)*measurement.sensor_position(0)*(x(State::speed)*x(State::speed))*2.0;
    H(Measurement::d2y_dt2, State::speed) = x(State::speed)*(x(State::dyaw_dx)*2.0-(x(State::droll_dx)*x(State::droll_dx))*measurement.sensor_position(1)-(x(State::dyaw_dx)*x(State::dyaw_dx))*measurement.sensor_position(1)+x(State::dpitch_dx)*x(State::droll_dx)*measurement.sensor_position(0)+x(State::dpitch_dx)*x(State::dyaw_dx)*measurement.sensor_position(2))*2.0;
    H(Measurement::d2y_dt2, State::accel) = -x(State::droll_dx)*measurement.sensor_position(2)+x(State::dyaw_dx)*measurement.sensor_position(0);
    H(Measurement::d2y_dt2, State::Roll) = measurement.gravity*cos(x(State::Pitch))*cos(x(State::Roll));
    H(Measurement::d2y_dt2, State::Pitch) = -measurement.gravity*sin(x(State::Pitch))*sin(x(State::Roll));
    H(Measurement::d2y_dt2, State::droll_dx) = -x(State::accel)*measurement.sensor_position(2)+x(State::dpitch_dx)*measurement.sensor_position(0)*(x(State::speed)*x(State::speed))-x(State::droll_dx)*measurement.sensor_position(1)*(x(State::speed)*x(State::speed))*2.0;
    H(Measurement::d2y_dt2, State::dpitch_dx) = (x(State::speed)*x(State::speed))*(x(State::droll_dx)*measurement.sensor_position(0)+x(State::dyaw_dx)*measurement.sensor_position(2));
    H(Measurement::d2y_dt2, State::dyaw_dx) = x(State::accel)*measurement.sensor_position(0)+(x(State::speed)*x(State::speed))*2.0+x(State::dpitch_dx)*measurement.sensor_position(2)*(x(State::speed)*x(State::speed))-x(State::dyaw_dx)*measurement.sensor_position(1)*(x(State::speed)*x(State::speed))*2.0;
    H(Measurement::d2z_dt2, State::speed) = x(State::speed)*(x(State::dpitch_dx)*2.0+(x(State::dpitch_dx)*x(State::dpitch_dx))*measurement.sensor_position(2)+(x(State::droll_dx)*x(State::droll_dx))*measurement.sensor_position(2)-x(State::dpitch_dx)*x(State::dyaw_dx)*measurement.sensor_position(1)-x(State::droll_dx)*x(State::dyaw_dx)*measurement.sensor_position(0))*-2.0;
    H(Measurement::d2z_dt2, State::accel) = -x(State::dpitch_dx)*measurement.sensor_position(0)+x(State::droll_dx)*measurement.sensor_position(1);
    H(Measurement::d2z_dt2, State::Roll) = -measurement.gravity*cos(x(State::Pitch))*sin(x(State::Roll));
    H(Measurement::d2z_dt2, State::Pitch) = -measurement.gravity*cos(x(State::Roll))*sin(x(State::Pitch));
    H(Measurement::d2z_dt2, State::droll_dx) = x(State::accel)*measurement.sensor_position(1)-x(State::droll_dx)*measurement.sensor_position(2)*(x(State::speed)*x(State::speed))*2.0+x(State::dyaw_dx)*measurement.sensor_position(0)*(x(State::speed)*x(State::speed));
    H(Measurement::d2z_dt2, State::dpitch_dx) = -x(State::accel)*measurement.sensor_position(0)-(x(State::speed)*x(State::speed))*2.0-x(State::dpitch_dx)*measurement.sensor_position(2)*(x(State::speed)*x(State::speed))*2.0+x(State::dyaw_dx)*measurement.sensor_position(1)*(x(State::speed)*x(State::speed));
    H(Measurement::d2z_dt2, State::dyaw_dx) = (x(State::speed)*x(State::speed))*(x(State::dpitch_dx)*measurement.sensor_position(1)+x(State::droll_dx)*measurement.sensor_position(0));
    H(Measurement::Roll, State::Roll) = 1.0;
    H(Measurement::Pitch, State::Pitch) = 1.0;
    H(Measurement::Yaw, State::Yaw) = 1.0;
    H(Measurement::droll_dt, State::speed) = x(State::droll_dx);
    H(Measurement::droll_dt, State::droll_dx) = x(State::speed);
    H(Measurement::dpitch_dt, State::speed) = x(State::dpitch_dx);
    H(Measurement::dpitch_dt, State::dpitch_dx) = x(State::speed);
    H(Measurement::dyaw_dt, State::speed) = x(State::dyaw_dx);
    H(Measurement::dyaw_dt, State::dyaw_dx) = x(State::speed);

    // clang-format on

    Eigen::MatrixXd R = measurement.R;
    Eigen::VectorXd y(MEASUREMENT_SIZE);
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        if (measurement.mask[i] && !isnan(measurement.z(i))) {
            y(i) = measurement.z(i) - h(i);

            if (R(i, i) < MIN_COVARIANCE) {
                R(i, i) = MIN_COVARIANCE;
            }
        } else {
            y(i) = 0.0;

            R.row(i).setConstant(0.0);
            R.col(i).setConstant(0.0);
            R(i, i) = INF_COVARIANCE;
        }
    }

    y(Measurement::Yaw) -= round(y(Measurement::Yaw) / (2 * M_PI)) * 2 * M_PI;

    Eigen::MatrixXd K =
        P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x.noalias() += K * y;
    constrain_state();
    P = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * P;
}

void AckermannEkf::constrain_state() {
    x.noalias() = x.cwiseMax(x_min_).cwiseMin(x_max_);
}
} // namespace ackermann_ekf