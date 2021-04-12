#ifndef ACKERMANN_EKF_ACKERMANN_EKF
#define ACKERMANN_EKF_ACKERMANN_EKF

#include <Eigen/Dense>

namespace ackermann_ekf {

struct Measurement {
  public:
    enum Index : int {
        X,
        Y,
        Z,
        dx_dt,
        dy_dt,
        dz_dt,
        d2x_dt2,
        d2y_dt2,
        d2z_dt2,
        Roll,
        Pitch,
        Yaw,
        droll_dt,
        dpitch_dt,
        dyaw_dt,
        SIZE
    };

    Eigen::VectorXd z;
    Eigen::MatrixXd R;

    std::array<bool, static_cast<int>(SIZE)> mask;

    double time = 0.0;

    double gravity = 9.82;

    Eigen::Vector3d sensor_position;

    Measurement()
        : z(static_cast<int>(SIZE)),
          R(static_cast<int>(SIZE), static_cast<int>(SIZE)) {
        z.setZero();
        R.setZero();
    }
};

struct ControlSignal {
  public:
    enum Index : int { speed, angle, SIZE };

    Eigen::VectorXd u;

    double time;

    ControlSignal() : u(static_cast<int>(SIZE)) {}
};

struct State {
  public:
    enum Index : int {
        X,
        Y,
        Z,
        speed,
        accel,
        Roll,
        Pitch,
        Yaw,
        droll_dx,
        dpitch_dx,
        dyaw_dx,
        SIZE
    };
};

class AckermannEkf {
  private:
    const Eigen::VectorXd x_min_, x_max_;

    ControlSignal control_signal_;
    bool control_signal_enabled_ = false;
    double wheelbase_;
    double control_acceleration_gain_, max_control_acceleration_;
    double control_angle_speed_gain_, max_control_angle_speed_;

    void constrain_state();

  public:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;

    double time = -1;

    AckermannEkf(const Eigen::VectorXd x_min, const Eigen::VectorXd x_max,
                 double wheelbase, double control_acceleration_gain,
                 double max_control_acceleration,
                 double control_angle_speed_gain,
                 double max_control_angle_speed);

    void process_measurement(const Measurement &measurement);
    void process_control_signal(const ControlSignal &control_signal);
    void bring_time_forward_to(double time);

    void predict(double dt);
    void correct(const Measurement &measurement);
};

const int STATE_SIZE = State::SIZE;
const int MEASUREMENT_SIZE = Measurement::SIZE;
const int CONTROL_SIGNAL_SIZE = ControlSignal::SIZE;
const int INF_COVARIANCE = 1e6;
const int MIN_COVARIANCE = 1e-6;

} // namespace ackermann_ekf

#endif