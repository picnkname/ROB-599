% Problem 1 State Estimation
clear
clc
%------------------------copy starts----------------------------------
%parameter definition
ms=400;
A=[0 -1;0 0];
B=[0;-1/ms];
F=[1;0];
C=[1 0];

%1.1 determine the observability matrix (O) and its rank (r)
O=[C; C*A];
r=rank(O);
%1.2 Pole placement Design
%1.2.1 Feedback Design. Find gain matrix, K, to place poles at -1,-1 
K=[0 1] * [B A*B]^-1 * (A^2+2*A+eye(2));
%1.2.2 Observer Design. Find gain matrix, G, to place poles at -3,-3 
G=(A^2+6*A+9*eye(2)) * [C; C*A]^-1 * [0; 1];

%1.2.3 Euler simulate to find the trajectories for 
%the system (x) and observer (x_hat)
t=0:1/100:10;
x_1=[0.1;0.1];
x_hat_1=[0;0];

% Generage white noise
N = length(t);
v = 0.0005 * randn(1,N);
w = 0.0005 * randn(1,N);
% Define new states z = [x; x_hat] and matrix A_z
z =  zeros(4,N);
A_z = [A     -B*K;
       G*C   A-G*C-B*K];
% Simulate the new system
z(:,1) = [x_1; x_hat_1];
for i = 1:N-1
 z(:,i+1) = z(:,i) + 0.01*(A_z*z(:,i) + [1;0;0;0]*w(i) + [0;0;G]*v(i)); % 0.01 is time interval
end
x= [z(1,:);z(2,:)];
x_hat= [z(3,:);z(4,:)];
% Show results (optional)
plot(t, z(1,:), t, z(2,:), t, z(3,:),t,z(4,:));
legend('x1', 'x2', 'xhat1', 'xhat2')
close all;

%1.3 LQR Design
%1.3.1 Feedback Design. use LQR to find feedback gains K_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Q=100*eye(2);
R=0.00005;

[K_lqr, P_1] = lqr_LTV(@(i)A,@(i)B,Q,R,t);

%1.3.2 Observer Design. use LQR to find optimal observer G_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Qo=eye(2);
Ro=1;

[G_lqr, P_2] = lqr_LTV(@(i)A',@(i)C',Qo,Ro,t);
for i = 1:length(G_lqr)
    G_lqr{i} = G_lqr{i}';
end

% 1.3.3 Euler Simulate
% Define new states z_lqr = [x_lqr; x_hat_lqr] and matrix A_z_lqr
z_lqr =  zeros(4,N);
A_z_lqr = [A'     -C'*G';
           K'*B'   A'-K'*B'-C'*G'];
% Simulate the new system
z_lqr(:,1) = [x_1; x_hat_1];
for i = 1:N-1
 z_lqr(:,i+1) = z_lqr(:,i) + 0.01*(A_z_lqr*z_lqr(:,i) + ...
     [1;0;0;0]*w(i) + [0;0;G]*v(i)); % 0.01 is time interval
end
x_lqr= [z_lqr(1,:);z_lqr(2,:)];
x_hat_lqr= [z_lqr(3,:);z_lqr(4,:)];
% Show results (optional)
plot(t, z_lqr(1,:), t, z_lqr(2,:), t, z_lqr(3,:),t,z_lqr(4,:));
legend('lqr:x1', 'lqr:x2', 'lqr:xhat1', 'lqr:xhat2')
close all


%1.4 Comparison
%Look at the observer error (x-x_hat) and (x_lqr-x_hat_lqr)
e_acker = x - x_hat;
e_lqr = x_lqr-x_hat_lqr;
hold on
plot(t,e_acker(1,:),'r')
plot(t,e_acker(2,:),'r');
plot(t,e_lqr(1,:),'b');
plot(t,e_lqr(2,:),'b');
legend('e1:acker','e2:acker', 'e1:lqr', 'e2:lqr');
hold off

%which observer converges faster? enter 'acker' or 'lqr'
%apostrophes included!
faster_convergence='acker';

%which observer is less noisy? enter 'acker' or 'lqr'
%apostrophes included!
less_noisy='lqr';

function [K, P] = lqr_LTV(AFun,BFun,Q,R,tSpan)
    nSteps = length(tSpan);

    P{nSteps} = zeros(size(Q));
    K{nSteps} = zeros(length(R),length(Q));
    
    for i = nSteps-1:-1:1
        A_ = AFun(i+1);
        B_ = BFun(i+1);
        P_ = P{i+1};
        
        P{i} = P_ + (tSpan(i+1)-tSpan(i)) * ( P_*A_ + A_'*P_ - P_*B_*(R\(B_'*P_)) + Q);
        K{i} = R\(B_'*P_);
    end
end
%------------------------copy ends----------------------------------