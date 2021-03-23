import numpy as np
import ackermann_ekf.generated.ekf_functions as ekf_functions


class State:
    IDX_X = 0
    IDX_Y = 1
    IDX_Z = 2
    IDX_accel = 3
    IDX_speed = 4
    IDX_ROLL = 5
    IDX_PITCH = 6
    IDX_YAW = 7
    IDX_droll_dx = 8
    IDX_dpitch_dx = 9
    IDX_dyaw_dx = 10
    SIZE = 11

    def __init__(self, x=None, P=None):
        self.x = np.zeros((State.SIZE, 1)) if x is None else np.asarray(x)

        if P is None:
            if x is None:
                self.P = np.eye(State.SIZE)*1e0
            else:
                self.P = np.eye(State.SIZE)*1e-6
        else:
            self.P = np.asarray(P)


class Measurement:
    IDX_X = 0
    IDX_Y = 1
    IDX_Z = 2
    IDX_accel = 3
    IDX_speed = 4
    IDX_ROLL = 5
    IDX_PITCH = 6
    IDX_YAW = 7
    IDX_droll_dt = 8
    IDX_dpitch_dt = 9
    IDX_dyaw_dt = 10
    SIZE = 11

    def __init__(self, z, R, time, sensor_position):
        self.z = np.asarray(z)
        self.R = np.asarray(R)
        self.time = float(time)
        self.sensor_position = np.asarray(sensor_position)


class EKF:
    def __init__(self,
                 time,
                 state=None,
                 Q=None,
                 Q_cutoff=None,
                 x_min=None,
                 x_max=None):
        self.time = time
        self.state = State() if state is None else state
        self.Q_cutoff = None
        self.x_min = None if x_min is None else np.asarray(x_min)
        self.x_max = None if x_max is None else np.asarray(x_max)

        if Q is None:
            self.Q = np.zeros((State.SIZE, State.SIZE))
            self.Q[State.IDX_X, State.IDX_X] = 1e-3
            self.Q[State.IDX_Y, State.IDX_Y] = 1e-3
            self.Q[State.IDX_Z, State.IDX_Z] = 1e-3
            self.Q[State.IDX_speed, State.IDX_speed] = 1e-2
            self.Q[State.IDX_accel, State.IDX_accel] = 1e-1
            self.Q[State.IDX_ROLL, State.IDX_ROLL] = 1e-2
            self.Q[State.IDX_PITCH, State.IDX_PITCH] = 1e-2
            self.Q[State.IDX_YAW, State.IDX_YAW] = 1e-2
            self.Q[State.IDX_droll_dx, State.IDX_droll_dx] = 1e-1
            self.Q[State.IDX_dpitch_dx, State.IDX_dpitch_dx] = 1e-1
            self.Q[State.IDX_dyaw_dx, State.IDX_dyaw_dx] = 1e-1
        else:
            self.Q = np.asarray(Q)

    def process_measurement(self, measurement):
        dt = measurement.time - self.time

        if dt > 0:
            self.state.x, self.state.P = self.predict(dt)
            self.time = measurement.time

        self.state.x, self.state.P, self.Q = self.correct(dt, measurement)

    def extrapolate_state(self, time):
        x = ekf_functions.f(self.state.x, dt=time - self.time)
        return x

    def predict(self, dt):
        x = ekf_functions.f(self.state.x, dt=dt)
        F = ekf_functions.F(self.state.x, dt=dt)
        P = F @ self.state.P @ F.T + self.Q * dt

        return self.validate(x), P

    def correct(self, dt, measurement):
        Q = self.Q

        hx = ekf_functions.h(self.state.x, measurement.sensor_position)
        y = measurement.z - hx
        H = ekf_functions.H(self.state.x, measurement.sensor_position)
        S = H @ self.state.P @ H.T + measurement.R
        K = self.state.P @ H.T @ np.linalg.inv(S)
        Ky = K @ y
        x = self.state.x + Ky
        P = (np.eye(State.SIZE) - K @ H) @ self.state.P

        if self.Q_cutoff and self.Q_cutoff > 0 and dt > 0:
            alpha = 1.0 / (1 + self.Q_cutoff * dt)
            Q = alpha * Q + (1 - alpha) * (Ky @ Ky.T)

        return self.validate(x), P, self.Q

    def validate(self, x):
        if self.x_min is not None or self.x_max is not None:
            return np.clip(x, self.x_min, self.x_max)
        return x
