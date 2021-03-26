#!/usr/bin/python3

import inspect
import sys

try:
    from sympy import *
except:
    print('Please install sympy first using\n\n    '
          'pip3 install sympy\n', file=sys.stderr)
    sys.exit(1)


def rpy2mat(roll, pitch, yaw):
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)

    return Matrix([[cy*cp, cy*sp*sr - cr*sy, sy*sr + cy*cr*sp],
                   [cp*sy, cy*cr + sy*sp*sr, cr*sy*sp - cy*sr],
                   [-sp,             cp*sr,            cp*cr]])


def mat2rpy(M):
    return atan2(M[2, 1], M[2, 2]), -asin(M[2, 0]), atan2(M[1, 0], M[0, 0])


def sympy2defstr(args, expr, name):
    return inspect.getsource(lambdify(args, expr, 'math')) \
        .replace('_lambdifygenerated', name) \
        .replace('ImmutableDenseMatrix', 'array')


var('X Y Z v a ROLL PITCH YAW droll_dx dpitch_dx dyaw_dx')
var('sensor_x sensor_y sensor_z')
var('dt')

v_vec = Matrix([v, 0, 0])
a_vec = Matrix([a, 0, 0])
XYZ = Matrix([X, Y, Z])
sensor_xyz = Matrix([sensor_x, sensor_y, sensor_z])

print('Calculating M...')
M = rpy2mat(ROLL, PITCH, YAW)

print('Calculating M1...')
M1 = M*rpy2mat(droll_dx*v*dt, dpitch_dx*v*dt, dyaw_dx*v*dt)

print('Calculating dM_dt...')
dM_dt = ((M1 - M)/dt).limit(dt, 0)

print('Calculating RPY1...')
ROLL1, PITCH1, YAW1 = mat2rpy(M1)

print('Calculating XYZ1...')
XYZ1 = XYZ + M*v_vec*dt + (dM_dt*v_vec + M*a_vec)*dt**2/2

state = Matrix([XYZ,
                v,
                a,
                ROLL,
                PITCH,
                YAW,
                droll_dx,
                dpitch_dx,
                dyaw_dx])

print('Calculating f...')
f = simplify(Matrix([series(XYZ1[0], dt, n=3).removeO(),
                     series(XYZ1[1], dt, n=3).removeO(),
                     series(XYZ1[2], dt, n=3).removeO(),
                     series(v + a*dt, dt, n=2).removeO(),
                     series(a, dt, n=1).removeO(),
                     series(ROLL1, dt, n=2).removeO(),
                     series(PITCH1, dt, n=2).removeO(),
                     series(YAW1, dt, n=2).removeO(),
                     series(droll_dx, dt, n=1).removeO(),
                     series(dpitch_dx, dt, n=1).removeO(),
                     series(dyaw_dx, dt, n=1).removeO()]))

print('Calculating F...')
F = simplify(f.jacobian(state))

omega = Matrix([droll_dx*v, dpitch_dx*v, dyaw_dx*v])
domega_dt = Matrix([droll_dx*a, dpitch_dx*a, dyaw_dx*a])

print('Calculating h...')
h = simplify(Matrix([XYZ + M*sensor_xyz,
                     v_vec + omega.cross(sensor_xyz),
                     a_vec + 2*omega.cross(v_vec)
                     + domega_dt.cross(sensor_xyz)
                     + omega.cross(omega.cross(sensor_xyz)),
                     ROLL,
                     PITCH,
                     YAW,
                     omega]))

print('Calculating H...')
H = simplify(h.jacobian(state))


print('Done!\n\n\n')
print(f"""
# =============================================================================
# | This document was autogenerated by python from                            |
# | {'{:<74}'.format(__file__)}|
# | EDITING THIS FILE BY HAND IS NOT RECOMMENDED                              |
# =============================================================================

from numpy import array
from math import *

{sympy2defstr([list(state)] + [dt], list(f), 'f')}

{sympy2defstr([list(state)] + [dt], F.tolist(), 'F')}

{sympy2defstr([list(state)] + [list(sensor_xyz)], list(h), 'h')}

{sympy2defstr([list(state)] + [list(sensor_xyz)], H.tolist(), 'H')}
""")
