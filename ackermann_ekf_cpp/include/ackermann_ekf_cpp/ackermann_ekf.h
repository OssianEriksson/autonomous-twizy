#ifndef ACKERMANN_EKF_ACKERMANN_EKF
#define ACKERMANN_EKF_ACKERMANN_EKF

#include <Eigen/Dense>
#include <math.h>

namespace ackermann_ekf {
struct Measurement {
    Eigen::VectorXd z_;

    Eigen::MatrixXd R_;

    double time_;

    Eigen::Vector3d sensor_position_;

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
};

struct State {
  public:
    enum StateIndex : int {
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
    Eigen::VectorXd x_;

    Eigen::MatrixXd P_;

    Eigen::MatrixXd Q_;

    Eigen::MatrixXd I_;

    double time_;

  public:
    AckermannEkf();

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