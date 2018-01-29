% Problem 2 LQR Control
clear
clc
%------------------------copy starts----------------------------------
d=1.5;
l=3;
gam=atan(0.3);
v=2;
psi=@(t) (v/l*tan(gam))*t;

% 1.1 Simulate the non-linear system to generate the circular trajectory
f=@(t,x)[v/l*(l*cos(x(3))-d*sin(x(3))*tan(gam)); ...
         v/l*(l*sin(x(3))+d*cos(x(3))*tan(gam)); ...
         v/l*tan(gam)];
T=0:0.01:5;

[T,Y]=ode45(f,T,[0,0,0]);

%1.2 Find the linearized time varying A and B matricies
A=@(t) [0   0   -v*sin(0.2*t)-v*d/l*tan(gam)*cos(0.2*t);...
        0   0   v*cos(0.2*t)-v*d/l*tan(gam)*sin(0.2*t);...
        0   0   0];
    
B=@(t) [cos(0.2*t)-d/l*sin(0.2*t)*tan(gam)  -v*d/l*sin(0.2*t)*(sec(gam))^2;...
        sin(0.2*t)+d/l*cos(0.2*t)*tan(gam)  v*d/l*cos(0.2*t)*(sec(gam))^2;...
        1/l*tan(gam)                        v/l*(sec(gam))^2];

%1.3 Find the optimal feedback gains
Q=eye(3);
R=eye(2);

%the function lqr_LTV requires the A and B matrix functions to be functions of the step number i
A1=@(i) A(T(i));
B1=@(i) B(T(i));
[K,P]=lqr_LTV(A1,B1,Q,R,T);

%1.4
x0=[0.10; 0.08; 0.01];

%similarly to lqr_LTV this function requires f to be a function of the step i
f1=@(i,dx)((A(i)-B(i)*K{i})*dx);
Y1=ode1(f1,T,x0)+Y;

%Plot(optinal)
figure(1)
plot(T,Y(:,1),T,Y(:,2),T,Y(:,3));
legend('x1','y1','psi1');
grid on
figure(2)
plot(T,Y1(:,1),T,Y1(:,2),T,Y1(:,3));
legend('x2','y2','psi2');
grid on

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

function x = ode1(odeFun, tSpan, x0)
    x = zeros(length(tSpan),length(x0));

    x(1,:) = x0';
    
    for i = 1:length(tSpan)-1
        x(i+1,:) = x(i,:) + (tSpan(i+1) - tSpan(i))*odeFun(i,x(i,:)')';
    end
end
%------------------------copy ends----------------------------------