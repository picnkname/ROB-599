function [x, u] = mpc_LTV( AFun, BFun, Q, R, tspan, PredHorizon, AXcons, bXcons, AUcons, bUcons, x0 )

n = size( x0, 1 ); %size of state vector
m = size( R, 1 ); % size of input vector
dt = tspan( 2 ) - tspan( 1 ); % assumed fixed time span

x = zeros( length( tspan ), n );
u = zeros( length( tspan ) - 1 , m ); % don't have an input at the last time step.

% the first n * PredHorizon variables of the decision variables will correspond to the state 
% the next m * (PredHorizon - 1) variables of the decision variables will correspond to the input

% we begin by defining the cost function
H = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ) );
c = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ), 1 );
for i = PredHorizon
    H( ( ( i - 1 ) * n + 1 ):( i * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = Q;
end
for i = 1:( PredHorizon - 1 )
    H( ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ), ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ) ) = R;
end

% next we define the inequality constraints corresponding to bound
% constraints we have 2 constraints for each state and input at each time
% instance
Aineq = zeros( 2 * n * PredHorizon + 2 * m * (PredHorizon - 1 ), n * PredHorizon + m * ( PredHorizon - 1 ) );
bineq = zeros( 2 * n * PredHorizon + 2 * m * (PredHorizon - 1 ), 1 );
% state constraints
for i = 1:PredHorizon
   Aineq( ( ( i - 1 ) * n * 2 + 1 ):( i * n * 2 ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = AXcons;
   bineq( ( ( i - 1 ) * n * 2 + 1 ):( i * n * 2 ), 1 ) = bXcons;
end
% input constraints
for i = 1:( PredHorizon - 1 )
    Aineq( ( ( i - 1 ) * m * 2 + 2 * n * PredHorizon + 1 ):( i * m * 2 + 2 * n * PredHorizon ), ...
        ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ) ) = AUcons;
    bineq( ( ( i - 1 ) * m * 2 + 2 * n * PredHorizon + 1 ):( i * m * 2 + 2 * n * PredHorizon ), 1 ) = bUcons;
end

x( 1, : ) = x0;
initial_state  = x0;
for j = 0:(length( tspan ) - 2)
    
    % next we define the equality constraints corresponding to Euler
    % integration one for the initial condition and one for each time step
    % until the next to last time step since the dynamics are time-varying we
    % will have to build this right before we solve the QP
    Aeq = zeros( n * PredHorizon, n * PredHorizon + m * ( PredHorizon - 1 ) );
    beq = zeros( n * PredHorizon, 1 );
       
    Aeq( 1:n, 1:n ) = eye( n ); 
    beq( 1:n ) = initial_state;
    for i = 1:( PredHorizon - 1 )
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( i * n + 1 ):( ( i + 1 ) * n ) ) = -eye( n );
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = dt * AFun( ( j + 1 - 1 ) * dt ) + eye( n );
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ...
            ( ( i - 1 ) * m + PredHorizon * n + 1 ):( i * m + PredHorizon * n ) ) = dt * BFun( ( j + 1 - 1 ) * dt );
    end
    
    solDV = quadprog( H, c, Aineq, bineq, Aeq, beq );
    
    x( j + 2, : ) = solDV( ( n + 1 ):( 2 * n ) );
    u( j + 1, : ) = solDV( ( n * PredHorizon + 1 ):( m + n * PredHorizon ) );
    initial_state = x( j + 2, : );
    
end
