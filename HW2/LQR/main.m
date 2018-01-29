%% Add paths
addpath('Utilities')

%% LTV lane return (2d Finite horizon)
sPrev = rng; % Generate a new seed
rng(sPrev);

Q = diag([1,1]); 
R = 10;

[tSpan,K] = LaneReturn(Q,R)

i = length(K);
clc
%% Display K iteratively
fprintf('\nK(t = %3.1f) = [%4.3f   %4.3f]\n',tSpan(i),K{i}(1),K{i}(2))
i = max(i-100,1);

%% Lane keeping
sPrev = rng; % Generate a new seed
rng(sPrev);
    
Q = diag([1,1]); 
R = 1;

LaneKeeping(Q,R)

%% Plot trajectory for pullout

TrajectoryFollowing( Q,R,'pullout',[],1 )


%% Pullout
sPrev = rng; % Generate a new seed

rng(sPrev);
    
Q = diag([1,1,1]); 
R = diag([1,1]);

TrajectoryFollowing( Q,R,'pullout' )

%% Plot trajectory for lane change

TrajectoryFollowing( Q,R,'lanechange',[],1 )


%% Lane change

sPrev = rng; % Generate a new seed

rng(sPrev);
    
Q = diag([1,1,1]); 
R = diag([1,1]);

TrajectoryFollowing( Q,R,'lanechange' )