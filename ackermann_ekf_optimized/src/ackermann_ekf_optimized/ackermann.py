from ackermann_ekf_optimized.symbolic import from_generated_file
from sympy import lambdify, Matrix, Symbol
from pathlib import Path
import numpy as np

generated_path = Path('ackermann')

_f = from_generated_file(generated_path / 'f.txt')
_F = from_generated_file(generated_path / 'F.txt')
_h = from_generated_file(generated_path / 'h.txt')
_H = from_generated_file(generated_path / 'H.txt')
_x = from_generated_file(generated_path / 'x.txt')
_z = from_generated_file(generated_path / 'z.txt')
_dt = from_generated_file(generated_path / 'dt.txt')
_sensor_position = from_generated_file(generated_path / 'sensor_position.txt')


def generate_f():
    f_lambda = lambdify([list(_x)] + [_dt], _f)
    F_lambda = lambdify([list(_x)] + [_dt], _F)
    return (lambda x, dt: f_lambda(x, dt).T[0]), F_lambda


def generate_h(mask, sensor_position):
    if isinstance(mask, np.ndarray):
        mask = mask.tolist()
    s = [(s0, s1) for s0, s1 in zip(_sensor_position, sensor_position)]
    h_lambda = lambdify([list(_x)], _h[mask, :].subs(s))
    H_lambda = lambdify([list(_x)], _H[mask, :].subs(s))
    return (lambda x: h_lambda(x).T[0]), H_lambda


def _mask(vector, name):
    symbol = Symbol(name)
    return np.asarray([a == symbol for a in vector])


def _mask(vector):
    class Mask(object):
        pass
    for i in range(len(vector)):
        m = np.asarray([False] * len(vector))
        m[i] = True
        setattr(Mask, str(vector[i]), m)
    return Mask

z_size = len(_z)
x_size = len(_x)

z_mask = _mask(_z)
x_mask = _mask(_x)

f, F = generate_f()