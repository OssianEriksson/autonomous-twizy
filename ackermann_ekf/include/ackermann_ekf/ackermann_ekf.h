#ifndef ACKERMANN_EKF_ACKERMANN_EKF
#define ACKERMANN_EKF_ACKERMANN_EKF

#include <Eigen/Dense>
#include <math.h>

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
  public:
    Eigen::VectorXd x;

    Eigen::MatrixXd P;

    Eigen::MatrixXd Q;

    double time;

    AckermannEkf(double time);

    void process_measurement(const Measurement &measurement);

    void predict(double dt);

    void correct(const Measurement &measurement);
};

const int STATE_SIZE = State::SIZE;
const int MEASUREMENT_SIZE = Measurement::SIZE;
const int INF_COVARIANCE = 1e6;
const int MIN_COVARIANCE = 1e-6;
} // namespace ackermann_ekf

#endif