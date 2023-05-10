function [xdot] = ctrle_func(x, t, A, B, Q, R)

% State Vector 
x1 = x(1); %posisi_x
x2 = x(2); % kecepatan_x_xdot
x3 = x(3); %posisi_y
x4 = x(4); % kecepatan_y_ydot
x5 = x(5); %posisi_z
x6 = x(6); %kecepatan_z_zdot
x7 = x(7); %posisi_roll
x8 = x(8); %kecepatan_roll
x9 = x(9); %posisi_pitch
x10 = x(10); %kecepatan_pitch
x11 = x(11); %posisi_yaw
x12 = x(12); %kecepatan_yaw

x = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12]; 

%% Trajectory 
% "Trajectory" (q_des) tetap mengikuti definisi dari state variable (q). 
% perubahan q_des terhadap waktu, tentunya d/dt(q_des) /= 0. 
r0 = 0;
k = 5;
f = 0.1;
z_rate = 5;
x_des_5 = z_rate*t;
x_des_6 = z_rate;
r = 5
x_des_1 = r*cos(2*pi*f*t);
x_des_2 = -2*pi*f*r*sin(2*pi*f*t);
x_des_3 = r*sin(2*pi*f*t);
x_des_4 = 2*pi*f*r*cos(2*pi*f*t);

x_des = [x_des_1,x_des_2,x_des_3,x_des_4, x_des_5, x_des_6, 0, 0, 0, 0, 0, 0]';

%% System Def
% u(t) system
K_lqr = lqr(A,B,Q,R);
u = -K_lqr*(x-x_des);

% Function output
xdot = A*x+(B*u);

