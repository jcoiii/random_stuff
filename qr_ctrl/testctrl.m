clc;
clear;
close all;

%% Parameter Sistem
% Dynamic Constants
g = 9.81;
m = 0.5; 
Ix = 0.005; 
Iy = 0.005; 
Iz = 0.005;
b =3.13e-5;
d =7.5e-7;
l = 0.2;

% Control Constraints
Q = 1*eye(12);
R = 1;

% Trajectory Constraints
r0 = 0;
k = 5;
f = 0.1;
z_rate = 5;

% ODE Definition
tspan = 0:0.01:50; 
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%% State Space
A = [
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 g 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 -g 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0
     ];

B = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    1/m 0 0 0;
    0 0 0 0;
    0 1/Ix 0 0;
    0 0 0 0;
    0 0 1/Iy 0;
    0 0 0 0;
    0 0 0 1/Iz
     ];

C = [
    1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0
    ];

%% Cek Keterkontrolan dan Keteramatan
ctrl = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B (A^6)*B (A^7)*B (A^8)*B (A^9)*B (A^10)*B (A^11)*B];
obs = [C; C*A; C*(A^2); C*(A^3); C*(A^4); C*(A^5); C*(A^6); C*(A^7); C*(A^8); C*(A^9); C*(A^10); C*(A^11)];

%% Uji Kestabilan Sistem
eig_sys = eig(A);
display(eig_sys);

%% ODE operation
[t,x]=ode45(@(t,x) ctrle_func(x, t, A, B, Q, R),tspan,x0);

%% Generate Circular Trajectory
q_des_5 = z_rate*t(:,1);
r = 5;
q_des_1 = r.*cos(2*pi*f*t(:,1));
q_des_3 = r.*sin(2*pi*f*t(:,1));

%% Plot
figure(1)
plot3(x(:,1),x(:,3),x(:,5))
hold on
plot3(q_des_1,q_des_3,q_des_5,'-o')
xlabel('Sumbu X')
ylabel('Sumbu Y')

% figure(2);
% plot(tspan,error_x);hold on;
% xlabel('tspan')
% ylabel('error x')
% 
% figure(3);
% plot(tspan,error_y);hold on;
% xlabel('tspan')
% ylabel('error y')



