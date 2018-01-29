pnt_2D = transform3Dto2D(1000);

function pnt_2D = transform3Dto2D(dim)

clear
clc

% Generate all transformation 
% 1. Translate by 250 along the world x-axis and 250 along y-axis
T_1 = [250; 250; 0];
M_1 = [eye(3,3)     T_1;
       zeros(1,3)   1];

% 2. Rotate by 105 degs CCK about world y-axis; 
%    Scale by 50 in world x direction and 0.25 in z direction.
theta = 105*pi/180;
R_2 = [cos(theta)     0      sin(theta);
       0              1      0;
       -sin(theta)    0      cos(theta)];
T_2 = [0; 0; 0];
S_2 = diag([50 1 0.25]);
M_2 = [S_2*R_2      T_2;
       zeros(1,3)   1];

% 3. Translate by 350 along current y-axis
T_3 = R_2 * [0; 350; 0];
M_3 = [eye(3,3) 	T_3
       zeros(1,3)   1];

% 4. Rotate by 45 degrees CCK about the current  x-axis
ref = M_3 * M_2 * M_1 * [0; 0; 0; 1];
ref = ref(1:3);
theta = 45/180*pi;
% R_4 = [1,0,0;0,cos(theta3),-sin(theta3);0,sin(theta3),cos(theta3)];
n = M_3 * [R_2  T_2; zeros(1,3) 1] * M_1 * [1;0;0;0];
K = [0   -n(3)   n(2);
    n(3)  0      -n(1);
    -n(2) n(1)   0];
R_4 = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;
M_4 =[eye(3),ref; zeros(1, 3), 1] * [R_4, zeros(3, 1); zeros(1, 3), 1] *[eye(3), -ref; zeros(1, 3), 1];
% M_4 =  [R4,[0,0,0]';0,0,0,1];
    

% Project from the fixed 3D world frame to 2D image plane defined by
% the perspective matrix P and intrinsic matrix I
T = M_4 * M_3 * M_2 * M_1;
hourglass_points = gen_hourglass(100,10);
p_new = T * hourglass_points';

% 1. perspective projection
f = 1;
p_prime = f* p_new(1:3,:)./ ([1;1;1]* p_new(3,:)); 
alpha = (dim-1)/range(p_prime(1,:));
beta = (dim-1)/range(p_prime(2,:));
col = min(p_prime,[],2);
  
pnt_2D = [alpha,0,-min(p_prime(1,:))*alpha;0,beta,-min(p_prime(2,:))*beta;0,0,1]*p_prime;
  
   
trans_points = [1,0,0,0;0,1,0,0;0,0,1,0]*p_new;
world_pix_mat= trans_points./trans_points(3,:);
max_col = max(world_pix_mat,[],2);
min_col = min(world_pix_mat,[],2);
scale= (dim-1)./(max_col-min_col);
scale(3) = 1;
pnt2d = (world_pix_mat-min_col).*scale;
end