from sympy import *
import inspect


var('a v x y z R P Y dr_dx dp_dx dy_dx dt hx hy hz')


def rpy2mat(R, P, Y):
    cR, sR = cos(R), sin(R)
    cP, sP = cos(P), sin(P)
    cY, sY = cos(Y), sin(Y)

    return Matrix([[cY*cP, cY*sP*sR - cR*sY, sY*sR + cY*cR*sP],
                   [cP*sY, cY*cR + sY*sP*sR, cR*sY*sP - cY*sR],
                   [-sP,              cP*sR,            cP*cR]])


def mat2rpy(M):
    return atan2(M[2, 1], M[2, 2]), -asin(M[2, 0]), atan2(M[1, 0], M[0, 0])


def lambda2str(args, expr, name):
    return inspect.getsource(lambdify(args, expr, 'math')) \
        .replace('_lambdifygenerated', name) \
        .replace('ImmutableDenseMatrix', 'array')


M0 = rpy2mat(R, P, Y)
M1 = M0*rpy2mat(dr_dx*v*dt, dp_dx*v*dt, dy_dx*v*dt)
dM_dt = ((M1 - M0)/dt).limit(dt, 0)

R1, P1, Y1 = mat2rpy(M1)

V = Matrix([v, 0, 0])
A = Matrix([a, 0, 0])

pos_f = Matrix([x, y, z]) + M0*V*dt + (dM_dt*V + M0*A)*dt**2/2

X = Matrix([x, y, z, v, a, R, P, Y, dr_dx, dp_dx, dy_dx])
f = simplify(Matrix([series(pos_f[0], dt, n=3).removeO(),
                     series(pos_f[1], dt, n=3).removeO(),
                     series(pos_f[2], dt, n=3).removeO(),
                     series(v + a*dt, dt, n=2).removeO(),
                     series(a, dt, n=1).removeO(),
                     series(R1, dt, n=2).removeO(),
                     series(P1, dt, n=2).removeO(),
                     series(Y1, dt, n=2).removeO(),
                     series(dr_dx, dt, n=1).removeO(),
                     series(dp_dx, dt, n=1).removeO(),
                     series(dy_dx, dt, n=1).removeO()]))
F = simplify(f.jacobian(X))


pos_h = Matrix([x, y, z]) + M0*Matrix([hx, hy, hz])

h = simplify(Matrix([*pos_h, v, a, R, P, Y, dr_dx*v, dp_dx*v, dy_dx*v]))
H = simplify(h.jacobian(X))


Xargs = [[arg] for arg in list(X)]
hargs = [[hx], [hy], [hz]]

print(f"""
# =============================================================================
# | This document was autogenerated by python from                            |
# | {'{:<74}'.format(__file__)}|
# | EDITING THIS FILE BY HAND IS NOT RECOMMENDED                              |
# =============================================================================

from numpy import array
from math import *

{lambda2str([Xargs] + [dt], f, 'f')}

{lambda2str([Xargs] + [dt], F, 'F')}

{lambda2str([Xargs] + [hargs], h, 'h')}

{lambda2str([Xargs] + [hargs], H, 'H')}
""")
