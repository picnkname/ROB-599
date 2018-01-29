function TrajectoryFollowing( Q,R,trajName,x0,trajOnly )
%TRAJECTORYFOLLOWING Finite horizon LTV LQR example
%   TRAJECTORYFOLLOWING(Q,R,trajName) Computes the finite horizon optimal 
%   control for a linearization of a Dubin's car traveling along a time varying
%   trajectory.

if nargin < 3
    trajName = 'pullout';
end

%% Set up dynamics
% The state x = [x; y; theta] consists of the position of the car in
% the lane (x,y) and the direction of travel (theta). The control input 
% u = [phi; v] represents the wheel angle of the car (phi) and the speed
% of the car (v). Phi is saturated to be between +/-phi_max.

% Set the max steering angle (must be less than pi/2 to avoid locking the
% steering)
phiMax = pi/2 - 0.1; 

% The full nonlinear dynamics of the system
DynNL = @(x,u) u(1) * [cos(x(3))            ;
                       sin(x(3))            ;
                       tan(sat(u(2),phiMax))];
                   
% Get desired trajectory
switch lower(trajName)
    case 'lanereturn'
        tSpan = 0:.01:5;
        
        vHat = 3*ones(size(tSpan));
        phiHat = 0*tSpan;
        x0_ = [0;0;0];
    
    case 'lanechange'
        tSpan = 0:.01:2;
        
        vHat = 3*ones(size(tSpan));
        phiHat = 0.2*(1 - 2*(tSpan > 1) + (tSpan > 2)); % Step input in phi
        x0_ = [0;0;0];
        
%         thetaHat = @(t) t - 2*(t - 1).*(t > 1) + (t - 2).*(t > 2);
%         
%         xHat = @(t) v_ * (sin(min(t,1)) + (sin(1) - sin(2 - min(t,2))).*(t > 1) + (t > 2).*(t - 2));
%         yHat = @(t) v_ * (1 - cos(min(t,1)) + (-cos(1) + cos(2 - min(t,2))).*(t > 1));

    case 'pullout'
        tSpan = 0:.01:2;
        vHat = max(5*tSpan,3);
        phiHat = -.428*(1 - (tSpan > 1)); % Step input in phi
        x0_ = [0;0;pi/2];
end

% Integrate trajectory forward
x_ = ode1(@(i,x) DynNL(x,[vHat(i);phiHat(i)]), tSpan, x0_);
if nargin > 4 && trajOnly
    AnimateCar(tSpan,{x_}),return
end

xHat = x_(:,1);
yHat = x_(:,2);
thetaHat = x_(:,3);

% Linearizing about the trajectory yields the LTV system:
AFun = @(i) [0, 0, -vHat(i)*sin(thetaHat(i));
             0, 0,  vHat(i)*cos(thetaHat(i));
             0, 0,                         0];

BFun = @(i) [cos(thetaHat(i)),                            0;
             sin(thetaHat(i)),                            0;
             tan(phiHat(i)), vHat(i)*(1+tan(phiHat(i))^2)];

%% Solve for the optimal cost to go and control input
% If Q and R unspecified, set them to identity
if nargin < 2
    Q = eye(3);
    R = eye(2);
end

[K, ~] = lqr_LTV(AFun,BFun,Q,R,tSpan);

%% Simulate and animate the output
% If x0 is unspecified, set it to [1;0] and 10 respectively
if nargin < 4
%     x0 = [-.5;-.5;.3];
      x0 = x0_ + 2*rand(3,1)-1;
end

% Simulate the full nonlinear dynamics of the system with the optimal
% control: u = -K(t)*deltaX + uHat
x = ode1(@(i,x) DynNL( x, -K{i}*(x - [xHat(i); yHat(i); thetaHat(i)]) + [vHat(i); phiHat(i)] ), tSpan, x0);

AnimateCar(tSpan,{x_,x})

end

