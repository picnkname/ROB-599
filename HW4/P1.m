% Problem 1: Model Predictive Control (MPC)
close all;
clear
clc

% Ssystem Dynamics
AFun =  [0, 0.310625; 0, 0];
BFun =  [0; 1];
dt = 0.01;
tSpan = 0 : dt : 10;
x0 = [-1.25; 0];
x0_nominal = [-1; 0];
u_nominal = @(t) -0.3175 * sin(pi*t/10 - pi/2);

% MPC
% 1.1 Euler Discretization
A = eye(2) + dt * AFun;
B = dt * BFun;

% 1.2 Decision Variables
Ndec = double(2*11 + 1*10);

% 1.3 Equality Constraints
PredHorizon = 10;
Aeq = zeros(2*(PredHorizon+1), 2*(PredHorizon+1) + PredHorizon);
beq = zeros(2*(PredHorizon+1), 1);

Aeq(1:2, 1:2) = eye(2); 
beq(1:2) = x0 - x0_nominal;

for i = 1:PredHorizon
    Aeq((i*2+1):((i+1)*2), (i*2+1):((i+1)*2)) = -eye(2);
    Aeq((i*2+1):((i+1)*2), ((i-1)*2+1):(i*2)) = A;
    Aeq((i*2+1):((i+1)*2), ((i-1)+(PredHorizon+1)*2+1):(i+(PredHorizon+1)*2)) = B;
end

% 1.4 Inequality Constraints
Aineq = [eye(32); -eye(32)];
bineq = [0.5*ones(22,1); (10-0.3175)*ones(10,1); 0.5*ones(22,1); (10+0.3175)*ones(10,1)];

% 1.5 Generating a Solution
Q = [100, 0; 0, 100];
R = 0;

x = zeros(length(tSpan), 2);
u = zeros(length(tSpan) - 1 , 1); % don't have input at the last time step.

% Definethe cost function
H = zeros(2*(PredHorizon+1) + PredHorizon);
c = zeros(2*(PredHorizon+1) + PredHorizon, 1);
for i = PredHorizon+1
    H(((i-1)*2 + 1):(i*2), ((i-1)*2 + 1):(i*2)) = Q;
end
for i = 1:PredHorizon 
    H((i-1 + 2*(PredHorizon+1) + 1):(i + 2*(PredHorizon+1)), (i-1 + 2*(PredHorizon+1) + 1):(i + 2*(PredHorizon+1))) = R;
end

initial_state = x0' - x0_nominal';
x(1,:) = initial_state;

% Define the equality constraints corresponding to Euler integration 
% one for the initial condition and one for each time step
% until the next to last time step since the dynamics are time-varying we
% will have to build this right before we solve the QP
for j = 0:(length(tSpan)-2)

    beq(1:2) = initial_state';
    %bineq(23:32) = (10 - u_nominal(dt*j)) * ones(10,1);
    %bineq(55:64) = (10 + u_nominal(dt*j)) * ones(10,1);
  
    solDV = quadprog( H, c, Aineq, bineq, Aeq, beq );
    
    %x(j+2, :) = solDV(3:4);
    u(j+1) = solDV(23);
    x(j+2,:) = A*x(j+1,:)' + B*u(j+1);
    initial_state = x(j+2,:);
    
end

for i = 1:size(x,1)
    x(i,1) = x(i,1) + x0_nominal(1);
end


% Plots (optional)
figure; 
subplot(211)
plot(x(:,1), x(:,2)); xlabel('x'); ylabel('y');
subplot(212)
plot(tSpan(1:end-1), u); xlabel('time'); ylabel('u');

% To pass 1.3 & 1.4
Aeq(1:2, 1:2) = eye(2); 
beq(1:2) = x0 - x0_nominal;
Aineq = [eye(32); -eye(32)];
bineq = [0.5*ones(22,1); (10-0.3175)*ones(10,1); 0.5*ones(22,1); (10+0.3175)*ones(10,1)];