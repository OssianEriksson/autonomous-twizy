clc

syms X Y Z speed accel Roll Pitch Yaw droll_dx dpitch_dx dyaw_dx real
syms dx_dt dy_dt dz_dt d2x_dt2 d2y_dt2 d2z_dt2 droll_dt dpitch_dt dyaw_dt real
syms sensor_x sensor_y sensor_z real
syms dt real

assume(Roll > -pi & Roll < pi)
assume(Pitch > -pi/2 & Pitch < pi/2)
assume(Yaw > -pi & Yaw < pi)

speed_vec = [speed; 0; 0];
accel_vec = [accel; 0; 0];
omega = [droll_dx; dpitch_dx; dyaw_dx]*speed;
domega_dt = [droll_dx; dpitch_dx; dyaw_dx]*accel;
sensor_position = [sensor_x; sensor_y; sensor_z];

M = rpy2mat(Roll, Pitch, Yaw);
M1 = M*rpy2mat(omega(1)*dt, omega(2)*dt, omega(3)*dt);
dM_dt = limit((M1 - M)/dt, dt, 0);

[Roll1, Pitch1, Yaw1] = mat2rpy(M1);

XYZ = [X; Y; Z];

state = [XYZ;
         speed;
         accel;
         [Roll; Pitch; Yaw]; 
         [droll_dx; dpitch_dx; dyaw_dx]];

observables = [XYZ;
               [dx_dt; dy_dt; dz_dt];
               [d2x_dt2; d2y_dt2; d2z_dt2];
               [Roll; Pitch; Yaw]; 
               [droll_dt; dpitch_dt; dyaw_dt]];

f_complete = [XYZ + M*speed_vec*dt + (dM_dt*speed_vec + M*accel_vec)*dt^2/2;
              speed + accel*dt;
              accel;
              Roll1;
              Pitch1;
              Yaw1;
              droll_dx;
              dpitch_dx;
              dyaw_dx];

f = simplify([taylor2(state(1:3), f_complete(1:3), dt);
              taylor1(state(4), f_complete(4), dt);
              taylor0(state(5), f_complete(5), dt);
              taylor1(state(6:8), f_complete(6:8), dt);
              taylor0(state(9:11), f_complete(9:11), dt)]);

F = simplify(jacobian(f, state));

h = simplify([XYZ + M*sensor_position;
              speed_vec + cross(omega, sensor_position);
              accel_vec + 2*cross(omega, speed_vec) + cross(domega_dt, sensor_position) + cross(omega, cross(omega, sensor_position));
              Roll;
              Pitch;
              Yaw;
              omega]);

H = simplify(jacobian(h, state));







