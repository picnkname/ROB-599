function demo_rot3D()
%   Demo code showing how to rotate points givien axis of rotation
%   using Rodrigues' rotation formula.
%
%   Author: Ming-Yuan Yu, University of Michigan
%   Last modified: 09/27/2017

%% See the function `rot()` for more details 
ref0 = [randn(3, 1); 1];
n0 = [randn(3, 1); 0];

% normalize to length of 1
n0 = n0 / norm(n0, 2);

%% Setup a 3D plot
figure(1)
clf()

view(3)
axis vis3d
axis equal
axis(5 * [-1, 1, -1, 1, -1, 1])
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on

%% Vertices of a cuboid in homogeneous coordinate
v0 = [-1, -1, -1, -1,  1,  1,  1,  1
    -1,  1,  1, -1, -1,  1,  1, -1
    -1, -1,  1,  1, -1, -1,  1,  1
    1,  1,  1,  1,  1,  1,  1,  1];
v0 = [diag([1, 2, 3]), zeros(3, 1); zeros(1, 3), 1] * v0;

% faces of the cuboid
f = [1, 2, 3, 4
    2, 3, 7, 6
    5, 6, 7, 8
    1, 4, 8, 5
    3, 4, 8, 7
    1, 2, 6, 5];

%% Start to animate ...
fprintf('use `ctrl + c` to terminate\n')
k = 0;
while 1
    % get the transformation
    theta = mod(2 * k, 360) ;
    T = rot(ref0(1:3), n0(1:3), deg2rad(theta));
    
    % apply T to all the vertices
    v = T * v0;
    p = patch('faces', f,  'vertices', v(1:3, :)', ...
        'edgecolor', 0.5 * [1, 1, 1],  'facealpha', 0.5);
    
    % both ref and n should be unchanged after transformation
    ref = T * ref0;
    n = T * n0;
    a = [ref + n, ref - n];
    l = plot3(ref(1), ref(2), ref(3), 'ro', ...
        a(1, :), a(2, :), a(3, :), 'k', 'linewidth', 2);
    
    title(['\theta = ', num2str(theta), ' deg'])
    if theta
        pause(0.02)
    else
        pause(2)
    end
    delete(p)
    delete(l)
    
    k = k + 1;
end

end

function T = trans(t)
% t:
%   (3, 1) double
%   displacement vector
T = [eye(3), t; zeros(1, 3), 1];
end

function T = rot(ref, n, theta)
% ref:
%   (3, 1) double
%   a reference point on the rotation axis
% n:
%   (3, 1) double
%   direction of the rotation axis, length must be 1.0
% theta:
%   (1, 1) double
%   rotation angle

% https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
K = [0, -n(3), n(2); n(3), 0, -n(1); -n(2), n(1), 0];
R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;

T = trans(ref) * [R, zeros(3, 1); zeros(1, 3), 1] * trans(-ref);
end