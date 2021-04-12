/**
 * @file ackermann_ekf.h
 *
 * @brief Extended kalman filter for Ackermann vehicle
 *
 * This header contains definitions of measurement and control signal data
 * structures, indicies of components in state/measurement/control vectors as
 * well as the specification of the filter interface.
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_ACKERMANN_EKF
#define ACKERMANN_EKF_ACKERMANN_EKF

#include <Eigen/Dense>

namespace ackermann_ekf {

/**
 * @brief Representation of measurement
 *
 * See the <a
 * href="https://github.com/OssianEriksson/autonomous-twizy/tree/master/ackermann_ekf/README.md#coordinate-frames">package
 * documentation</a> for definition of the coordinate frames and Euler angle
 * conventions used.
 */
struct Measurement {
  public:
    /**
     * @brief Indecies of components in #z
     */
    enum Index : int {
        /**
         * x component of position vector in global frame
         */
        X,

        /**
         * y component of position vector in global frame
         */
        Y,

        /**
         * z component of position vector in global frame
         */
        Z,

        /**
         * Time derivative of x component of position vector in local frame
         * (forward velocity)
         */
        dx_dt,

        /**
         * Time derivative of y component of position vector in local frame
         */
        dy_dt,

        /**
         * Time derivative of z component of position vector in local frame
         */
        dz_dt,

        /**
         * Second time derivative of x component of position vector in local
         * frame (forward acceleration)
         */
        d2x_dt2,

        /**
         * Second time derivative of y component of position vector in local
         * frame
         */
        d2y_dt2,

        /**
         * Second time derivative of z component of position vector in local
         * frame
         */
        d2z_dt2,

        /**
         * Roll in global frame
         */
        Roll,

        /**
         * Pitch in global frame
         */
        Pitch,

        /**
         * Yaw in global frame
         */
        Yaw,

        /**
         * Time derivative of roll in local frame
         */
        droll_dt,

        /**
         * Time derivative of pitch in local frame
         */
        dpitch_dt,

        /**
         * Time derivative of yaw in local frame
         */
        dyaw_dt,

        /**
         * Size of measurement vector #z
         */
        SIZE
    };

    /**
     * The measurement vector (dim #MEASUREMENT_SIZE x 1)
     */
    Eigen::VectorXd z;

    /**
     * The measurement covariance matrix (dim #MEASUREMENT_SIZE x
     * #MEASUREMENT_SIZE)
     */
    Eigen::MatrixXd R;

    /**
     * Mask describing which components of the measurement vector contain actual
     * data to be integrated into the filter state. A value of true at index i
     * in the mask vector means #z
     */
    std::array<bool, static_cast<int>(SIZE)> mask;

    /**
     * The time of the measurement
     */
    double time = 0.0;

    /**
     * Gravitational acceleration associated with the measurement (is later
     * subtracted from acceleration measurement).
     */
    double gravity = 9.82;

    /**
     * Position of sensor in local frame
     */
    Eigen::Vector3d sensor_position;

    /**
     * Zero initialization of #z and #R.
     */
    Measurement()
        : z(static_cast<int>(SIZE)),
          R(static_cast<int>(SIZE), static_cast<int>(SIZE)) {
        z.setZero();
        R.setZero();
    }
};

/**
 * @brief Representation of a control signal
 */
struct ControlSignal {
  public:
    /**
     * @brief Indecies of components in #u
     */
    enum Index : int {
        /**
         * Desired forward velocity
         */
        speed,

        /**
         * Desired steering angle (positive for left turn)
         */
        angle,

        /**
         * Size of control vector #u
         */
        SIZE
    };

    /**
     * The control vector (dim #CONTROL_SIGNAL_SIZE x 1)
     */
    Eigen::VectorXd u;

    /**
     * Time for which the control signal was issued
     */
    double time;

    ControlSignal() : u(static_cast<int>(SIZE)) {}
};

/**
 * @brief Representation of a filter state
 *
 * See the <a
 * href="https://github.com/OssianEriksson/autonomous-twizy/tree/master/ackermann_ekf/README.md#coordinate-frames">package
 * documentation</a> for definition of the coordinate frames and Euler angle
 * conventions used.
 */
struct State {
  public:
    /**
     * @brief Indecies of components in state vector
     */
    enum Index : int {
        /**
         * x component of position vector in global frame
         */
        X,

        /**
         * y component of position vector in global frame
         */
        Y,

        /**
         * z component of position vector in global frame
         */
        Z,

        /**
         * Forward speed in local frame
         */
        speed,

        /**
         * Forward acceleration in local frame
         */
        accel,

        /**
         * Roll in global frame
         */
        Roll,

        /**
         * Pitch in global frame
         */
        Pitch,

        /**
         * Yaw in global frame
         */
        Yaw,

        /**
         * Derivative of roll in local frame with respect to forward position
         * (also in local frame)
         */
        droll_dx,

        /**
         * Derivative of pitch in local frame with respect to forward position
         * (also in local frame)
         */
        dpitch_dx,

        /**
         * Derivative of yaw in local frame with respect to forward position
         * (also in local frame)
         */
        dyaw_dx,

        /**
         * Size of state vector
         */
        SIZE
    };
};

/**
 * @brief Extended Kalman filter (EKF) for Ackermann vehicle
 */
class AckermannEkf {
  private:
    /**
     * Lower bound on state vector
     */
    const Eigen::VectorXd x_min_;

    /**
     * Upper bound on state vector
     */
    const Eigen::VectorXd x_max_;

    /**
     * Last seen control signal
     */
    ControlSignal control_signal_;

    /**
     * Whether to use #control_signal_ for state propagation
     */
    bool control_signal_enabled_ = false;

    /**
     * Wheelbase of Ackermann vehicle
     */
    double wheelbase_;

    /**
     * In predicting acceleration from control velocity, this is the gain
     * related to velocity error.
     */
    double control_acceleration_gain_;

    /**
     * In predicting acceleration from control velocity, this is the maximum
     * allowed acceleration: Accelerations above (or below the negated value)
     * will be clamped to this value.
     */
    double max_control_acceleration_;

    /**
     * In predicting steering angle velocity from control steering angle, this
     * is the gain related to steering angle error.
     */
    double control_angle_speed_gain_;

    /**
     * In predicting steering angle velocity from control steering angle, this
     * is the maximum allowed steering angle velocity: Velocities above (or
     * below the negated value) will be clamped to this value.
     */
    double max_control_angle_speed_;

    /**
     * Apply clamping by #x_min_ and #x_max_ to state vector #x
     */
    void constrain_state();

  public:
    /**
     * State vector (dim #STATE_SIZE x 1)
     */
    Eigen::VectorXd x;

    /*
     * State covariance matrix (dim #STATE_SIZE x #STATE_SIZE)
     */
    Eigen::MatrixXd P;

    /*
     * Process noise covariance matrix (dim #STATE_SIZE x #STATE_SIZE)
     */
    Eigen::MatrixXd Q;

    /**
     * The latest time filter state has been propagated to
     */
    double time = -1;

    AckermannEkf(const Eigen::VectorXd x_min, const Eigen::VectorXd x_max,
                 double wheelbase, double control_acceleration_gain,
                 double max_control_acceleration,
                 double control_angle_speed_gain,
                 double max_control_angle_speed);

    /**
     * Fuse a measurement value into filter state
     */
    void process_measurement(const Measurement &measurement);

    /**
     * Register a control signal for use in Kalman prediction step
     */
    void process_control_signal(const ControlSignal &control_signal);

    /**
     * Use Kalman prediction to bring filter state forward
     *
     * @param time The time to bring filter state forward to
     */
    void bring_time_forward_to(double time);

    /**
     * Perform Kalman prediction step
     *
     * @param dt How far into the future to predict
     */
    void predict(double dt);

    /**
     * Perform Kalman correction step using provided measurement
     */
    void correct(const Measurement &measurement);
};

/**
 * Size of state vector
 */
const int STATE_SIZE = State::SIZE;

/**
 * Size of measurement vector
 */
const int MEASUREMENT_SIZE = Measurement::SIZE;

/**
 * Size of control signal vector
 */
const int CONTROL_SIGNAL_SIZE = ControlSignal::SIZE;

/**
 * A representation of infinite covariance. The value can however not be too
 * large since this may lead to numerical instability
 */
const int INF_COVARIANCE = 1e6;

/**
 * A representation of zero covariance. The value can however not be too
 * small since this may lead to numerical instability
 */
const int MIN_COVARIANCE = 1e-6;

} // namespace ackermann_ekf

#endif