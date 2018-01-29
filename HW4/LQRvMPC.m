clear
clc
%% system dynamics
AFun = @(i) [ 1 1; 0 1 ];
BFun = @(i) [ 0; 1 ];
x0 = [ -1; -1 ];
totalSteps = 1000;
tSpan = linspace( 0, 5, totalSteps );
Q = [ 1e3 0; 0 1e3 ]; R = 0.1; 

%% LQR
[K, P] = lqr_LTV( AFun, BFun, Q, R, tSpan);

% generating data to be plotted
xlqr = ode1(@(i,x) ( AFun(i) - BFun(i)*K{i} ) * x, tSpan, x0 );
ulqr = zeros( totalSteps, 1 );
for i = 1:totalSteps
    ulqr( i ) = K{ i } * xlqr( i, : )';
end

figure( 1 ); 
subplot( 1, 2, 1 ); plot( xlqr( :, 1 ), xlqr( :, 2 ) ); xlabel( 'x1', 'FontSize', 20 ); ylabel( 'x2', 'FontSize', 20 ); title('LQR', 'FontSize', 20 ); set(gca,'fontsize',20)
subplot( 1, 2, 2 ); plot( tSpan, ulqr ); xlabel( 'time', 'FontSize', 20 ); ylabel( 'u', 'FontSize', 20 ); title('LQR', 'FontSize', 20 ); set(gca,'fontsize',20)

%% MPC
PredHorizon = 100;
AXcons = [ 1 0; -1 0; 0 1; 0 -1 ];
bXcons = [ 2; 2; 2; 2 ];
AUcons = [ 1; -1 ];
bUcons = [ 10; 10 ];

tic;
[ xmpc, umpc ] = mpc_LTV1( AFun, BFun, Q, R, tSpan, PredHorizon, AXcons, bXcons, AUcons, bUcons, x0 );
toc

figure( 2 ); 
subplot( 1, 2, 1 ); plot( xmpc( :, 1 ), xmpc( :, 2 ) ); xlabel( 'x1', 'FontSize', 20 ); ylabel( 'x2', 'FontSize', 20 ); title('MPC', 'FontSize', 20 ); set(gca,'fontsize',20)
subplot( 1, 2, 2 ); plot( tSpan( 1:end-1 ), umpc ); xlabel( 'time', 'FontSize', 20 ); ylabel( 'u', 'FontSize', 20 ); title('MPC', 'FontSize', 20 ); set(gca,'fontsize',20)
