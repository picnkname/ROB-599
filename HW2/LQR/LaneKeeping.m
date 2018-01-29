function LaneKeeping(Q,R,x0)
% LANEKEEPING Infinite horizon LTI LQR example
%   LANEKEEPING(Q,R) Computes the infinite horizon optimal control for a
%   linearization of a Dubin's car traveling at constant speed on a
%   straight road.

%% Set up dynamics
% The state x = [y; theta] consists of the lateral position of the car in
% the lane (y) and the direction of travel (theta). The control input 
% u = phi represents the wheel angle of the car (phi = 0 will maintain the
% car's current direction of travel). Phi is saturated to be between 
% +/-phi_max.

% Set the speed of travel
v = 5;
tf = 2;

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

[K, P] = lqr(A,B,Q,R);

fprintf(['\nThe infinite horizon cost to go of the system is given by: \n',...
           'V = [ y  theta ] * [ %6.3f  %6.3f ] * [   y   ]\n',...
           '                   [ %6.3f  %6.3f ]   [ theta ]\n'],...
         P(1,1),P(1,2),P(2,1),P(2,2))

fprintf(['\nThe optimal control input to the system is given by: \n',...
           'phi =  [ %6.3f  %6.3f ] * [   y   ]\n',...
           '                            [ theta ]\n'],...
           K(1),K(2))


%% Simulate and animate the output
% If x0 and tf are unspecified, set them to [1;0] and 10 respectively
if nargin < 3
      x0 = 2*rand(2,1)-1;
end

% Simulate the full nonlinear dynamics of the system with the optimal
% control: u = -K*x
[t,x] = ode45(@(~,x) DynNL( x, -K*x), [0,tf], x0);

h = figure(1);
clf
hold on
plot([0,t(end)*v], [0,0], '--r')

AnimateCar(t,x,v,h)

end
