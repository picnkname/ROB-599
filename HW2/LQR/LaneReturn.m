function [tSpan,K] = LaneReturn( Q,R,x0 )
%LANERETURN Finite horizon LTV LQR example
%   LANERETURN(Q,R,x0) Computes the finite horizon optimal 
%   control for a linearization of a Dubin's car traveling at constant 
%   speed on a straight road.

%% Set up dynamics
% The state x = [y; theta] consists of the lateral position of the car in
% the lane (y) and the direction of travel (theta). The control input 
% u = phi represents the wheel angle of the car (phi = 0 will maintain the
% car's current direction of travel). Phi is saturated to be between 
% +/-phi_max.

% Set the speed of travel
v = 5;
tSpan = 0:0.001:2;
 
% Set the max steering angle (must be less than pi/2 to avoid locking the
% steering)
phiMax = pi/2 - 0.01; 

% The full nonlinear dynamics of the system
DynNL = @(x,u) v * [sin(x(2))           ;
                    tan(sat(u,phiMax))];

% Linearizing about the point x = [0,0] u = 0; yields the linear system:
A = [0, v;
     0, 0];

B = [0;
     v];

%% Solve for the optimal cost to go and control input
% If Q and R unspecified, set them to identity
if nargin < 2
    Q = eye(2);
    R = 1;
end

[K, ~] = lqr_LTV(@(~) A, @(~) B,Q,R,tSpan);

%% Simulate and animate the output
% If x0 is unspecified, set it to [1;0] and 10 respectively
if nargin < 3
%     x0 = [-.5;-.5;.3];
      x0 = 2*rand(2,1)-1;
end

% Simulate the full nonlinear dynamics of the system with the optimal
% control: u = -K(t)*x
x = ode1(@(i,x) DynNL( x, -K{i}*x ), tSpan, x0);

h = figure(1);
clf
hold on
plot([0,tSpan(end)*v], [0,0], '--r')

AnimateCar(tSpan',x,v,h)

end

