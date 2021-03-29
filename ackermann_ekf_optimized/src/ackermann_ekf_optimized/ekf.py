import numpy as np


class Measurement:
    def __init__(self, z, R, h, H, time):
        self.z = np.asarray(z)
        self.R = np.asarray(R)
        self.h = h
        self.H = H
        self.time = float(time)

    def __lt__(self, other):
        return self.time < other.time


class EKF:
    def __init__(self, x, P, f, F, Q, time=0.0):
        self.measurements = []
        self.x =  np.asarray(x)
        self.P = np.asarray(P)
        self.f = f
        self.F = F
        self.Q = np.asarray(Q)
        self.time = time

    def process_measurement(self, measurement):
        dt = measurement.time - self.time
        if dt > 0:
            self.time = measurement.time
            self._predict(dt)

        self._correct(measurement)

    def _predict(self, dt):
        self.x = self.f(self.x, dt)
        F = self.F(self.x, dt)
        self.P = F @ self.P @ F.T + self.Q * dt

    def _correct(self, measurement):
        y = measurement.z - measurement.h(self.x)
        H = measurement.H(self.x)
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + measurement.R)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P
