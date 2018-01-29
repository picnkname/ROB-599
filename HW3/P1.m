%Problem 1 : Tire Forces: Mild Manuever
%clear
%clc

%Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =  2420.0;	% yaw moment of inertia (kg-m^2)
b=L-a;   %distance of c.g to rear axel (m) 
g=9.81;
vx=20;

%%Tire forces
B=10;
C=1.3;
D=1;
E=0.97;

%timespan for all simulations
T=0:0.01:1;

%1.1 compute front and rear cornerning stifness
F_zf = b*m*g/L;
F_zr = a*m*g/L;
Ca_r=a/L*m*g*C*(B*(1-E)+E*B);
Ca_f=b/L*m*g*C*(B*(1-E)+E*B);

%1.2.1 compute the front and rear cornering stifness for the vehicle generate equilibrium trajetory using Euler integration and linear tire
%forces
% Ca_r = F_zr*B*C*D;
% Ca_f = F_zf*B*C*D;
delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;
F_yr = @(x, delta) Ca_r*( -(x(1) - b*x(2))/vx );
F_yf = @(x, delta) Ca_f*( delta - ( x(1) + a*x(2))/vx );
dzdt = @(x, u) [
    vx*cos(x(3)) - x(4)*sin(x(3));
    x(4)*cos(x(3)) + vx*sin(x(3));
    x(5);
    ( F_yr([x(4), x(5)], u) + F_yf([x(4), x(5)], u) - m*vx*x(5) )/m;
    ( -b*F_yr([x(4), x(5)], u) + a*F_yf([x(4), x(5)], u) )/Iz];

% integrate trajectory forward
t_span = T(end)/(length(T)-1);
Z_eq(:,1) = zeros(5,1); 
for i = 1:length(T) - 1
    Z_eq(:,i+1) = Z_eq(:,i) + (T(i+1) - T(i))* ...
                    dzdt(Z_eq(:,i), delta_fun((i-1)*t_span));
end          

plot(T,Z_eq(2,:));

%1.2.2 linearization for feedback gains bike with linear tire forces
Q = eye(5);
R = 0.5;

psi_hat = Z_eq(3,:);
vy_hat = Z_eq(4,:);
r_hat = Z_eq(5,:);

Afun = @(i) [
    0 0 (-vx*sin(psi_hat(i))-vy_hat(i)*cos(psi_hat(i))) -sin(psi_hat(i)) 0;
    0 0 (-vy_hat(i)*sin(psi_hat(i))+vx*cos(psi_hat(i))) cos(psi_hat(i)) 0;
    0 0 0 0 1;
    0 0 0 -(Ca_r + Ca_f)/m/vx ((Ca_r*b - Ca_f*a)/m/vx - vx);
    0 0 0 (b*Ca_r - a*Ca_f)/Iz/vx -(b*b*Ca_r + a*a*Ca_f)/Iz/vx];      
Bfun = @(i) [0; 0; 0; Ca_f/m; a*Ca_f/Iz];

[K,~] = lqr_LTV(Afun, Bfun, Q, R, T);

%1.2.3 Plot linear vs nonlinear tire forces and find max % difference
% Linear
for i = 1:length(T)
    F_yf_linear(i) = Ca_f*(delta_fun(i*t_span) - (vy_hat(i)+a*r_hat(i))/vx);
    F_yr_linear(i) = -Ca_r*(vy_hat(i) - b*r_hat(i))/vx;
    
    alpha_f_Pac(i) = delta_fun(i*t_span) - atan((vy_hat(i)+a*r_hat(i))/vx);
    alpha_r_Pac(i) = atan(-(vy_hat(i) - b*r_hat(i))/vx);
    F_yf_Pac(i) = F_zf*D*sin(C*atan(B*(1-E)*alpha_f_Pac(i) + E*atan(B*alpha_f_Pac(i))));
    F_yr_Pac(i) = F_zr*D*sin(C*atan(B*(1-E)*alpha_r_Pac(i) + E*atan(B*alpha_r_Pac(i))));
end

figure;
plot(T,F_yf_linear,T,F_yf_Pac); 
figure;
plot(T,F_yr_linear,T,F_yr_Pac);

error(1) = 100*max( abs((F_yf_linear-F_yf_Pac)./F_yf_Pac) );
error(2) = 100*max( abs((F_yr_linear-F_yr_Pac)./F_yr_Pac) );
tireforce_percent_error = max(error);

%1.2.4 Euler Simulate with  Nonlinear tire dynamics
delta_max = 45/180*pi; 
alpha_f_NL = @(x, delta) sign(delta)*min(abs(delta),45/180*pi) - atan(( x(1) + a*x(2))/vx); 
alpha_r_NL = @(x, delta) -atan((x(1) - b*x(2))/vx);

F_yf_NL = @(x, delta) F_zf*D*sin(C*atan(B*(1-E)*alpha_f_NL(x, delta) + ...
    E*atan(B*alpha_f_NL(x, delta))));

F_yr_NL = @(x, delta) F_zr*D*sin(C*atan(B*(1-E)*alpha_r_NL(x, delta) + ...
    E*atan(B*alpha_r_NL(x, delta))));

dzdy_NL = @(x, u) [vx*cos(x(3)) - x(4)*sin(x(3));
                x(4)*cos(x(3)) + vx*sin(x(3));
                x(5);
                ( F_yr_NL([x(4), x(5)], u) + F_yf_NL([x(4), x(5)], u)*cos(u) - m*vx*x(5) )/m;
                ( -b*F_yr_NL([x(4), x(5)], u) + a*F_yf_NL([x(4), x(5)], u)*cos(u) )/Iz];
            
Z(:,1) = zeros(5,1);
for i = 1:length(T)-1
   delta_(i) = K{i}*(Z_eq(:,i) - Z(:,i)) + delta_fun((i-1)*t_span);
   Z(:,i+1) = Z(:,i) + (T(i+1) - T(i))*dzdy_NL(Z(:,i), delta_(i));
end
            
[max_distance_error, index] = max( sqrt((Z_eq(1,:)-Z(1,:)).^2 + (Z_eq(2,:)-Z(2,:)).^2))


%Function library


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

