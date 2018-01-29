%clear
%clc

%parameter definition
m=1500;
n=4.8;
r=0.4;
g=9.81;
C_r=0.01;
rho=1.3;
C_d=0.32;
a=2.4;
theta_e=2*pi/180;
v_e=20;

%you may enter equations, but do not change the names of the variables given. 
%The auto-grader uses these variable names to check your answers!

%Part 1.1 Determine equilibrium engine torque
u_e= r/n * (m*g*C_r + 0.5*rho*C_d*a*v_e*v_e + m*g*sin(theta_e))

%Part 1.2.1 Determine the A,B,F matrices for the linearized system
A= - rho * C_d * a * v_e / m
B= n / (m * r)
F= - g * cos(theta_e)

%Part 1.2.2 Determine feedback gains of linear system to place pole at -1
k= 1 * 1/B * (A+1)

%Part 1.2.3 Determine the steady state error after 10 seconds

% Parameters
v0 = -1;    %initial condition
t = 0 : 0.01 : 10;  %simulation time
theta = [0 3*pi/180];   %theta in radians

% Simulate the system
odefun_1 = @(t,v)((A-B*k)*v + F*theta(1)); %without disturbance
odefun_2 = @(t,v)((A-B*k)*v + F*theta(2)); %with disturbance

% ODE of the feedback system
[t1,v1] = ode45(odefun_1, t , v0); %without disturbance
[t2,v2] = ode45(odefun_2, t , v0); %with disturbance

% Show results
plot(t1,v1,t2,v2)   
legend('theta = 0 deg, no steady state error',...
    'theta = 3 deg, has steady state error')
grid on

% Output the answer
sse= v2(end)

%Part 1.3.1 Determine A_I,B_I,F_I matrices of the linear system with integral action
%the subscript I is just used to indicate these matrices and vectors apply 
A_I= [A 0; 1 0]
B_I= [B ;0]
F_I= [F ;0]

%Part 1.3.2 Place poles of the system with integral action at -1,-2
k_I= [0 1] * [B_I A_I*B_I]^-1 *(A_I^2+3*A_I+2*eye(2))

%Part 1.3.3 with integral action determine the steady state error after 10
%seconds

% Parameters
v0 = [-1 ;0];   %v(0)=-1, z(0)=0

% Simulate the integral system
odefun = @(t,v)((A_I-B_I*k_I)*v + F_I*theta(2)); %with disturbance
[t3,v3] = ode45(odefun,t,v0);

% Show results
figure
plot(t3, v3(:,1))   
legend('v_t_i_l_d_e with theta = 3 deg under integral action')
grid on

sse_with_integral_action= v3(end,1)

