clear
clc

A=[-3 0 0;0 -1 -3;0 1 -2];

%2.1
[V,D] = eig(A);
l1= D(1,1);
l2= D(2,2);
l3= D(3,3);

%List eigenvectors such that v1 is the eigenvector for eigenvalue l1.
%Eigenvectors should be in column vector format.
v1=[V(:,1)];
v2=[V(:,2)];
v3=[V(:,3)];

%2.2
%Write out the close form equation using the symbolic variable t as time
syms t
x0= [5; 0; 0];
x= expm(A*t)*x0;
%x= x(1) % Variable x must be of size [3 1]

% 2.3
f=@(t,x)(A*x) ;
tspan=[0:0.1:10];
x0=[0 ; 1 ; 2];

[T,Y]=ode45(f,tspan,x0);

%Plot results to check
plot(T, Y(:,1), T, Y(:,2), T, Y(:,3))
legend('x1','x2','x3')
grid on

%2.4 
interval = 0.1;
integration = 0;
v = [1; 0; 0];
x_t = Y';
for i = 1 : length(tspan)
    integration = integration + x_t(:,i)' * v * interval;
end

sol24= integration