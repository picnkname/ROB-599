% Problem 3: 2D Affine Tranformations in Image Processing
clear
clc

% Load original apple
apple = rgb2gray(imread('cartoonapple.jpeg'));
apple = double(apple);  %convert to double

% 0.Initialize new apple
newapple = zeros(500,500);

% 1. Translation by (250, 200) along the current frame
T_1 = [1 0 250;
       0 1 200;
       0 0 1 ];

% 2. Rotation about current frame z-axis by -90 degrees
theta = -90*pi/180;
R_2 = [cos(theta) -sin(theta)  0;
       sin(theta)  cos(theta)  0;
       0           0           1];
   
% 3. Translation by 100 along current frame y-axis
ty = 100;
T_2 = [1 0 0;
      0 1 ty;
      0 0 1 ];

% 4. Rotate about world frame z-axis by 45 degrees
%theta = 0*pi/180;
theta = 45*pi/180;
R_4 = [cos(theta) -sin(theta)  0;
       sin(theta)  cos(theta)  0;
       0           0           1];

% 5. Scale by 2 times in the current frame
%s = 1;
s = 2;
S_5 = [s 0 0;
       0 s 0;
       0 0 1];

% 6. Translate by -150 along the world frame y-axis
%ty = 0;
ty = -150;
T_6 = [1 0 0;
       0 1 ty;
       0 0 1];

% Generate new apple
H = T_6 * R_4 * T_1 *  R_2 * T_2 * S_5;
%
for x = 0:499
    for y = 0:499
        trans_pos = inv(H) * [x; y; 1];
        X = round(trans_pos(1));
        Y = round(trans_pos(2));
        if(X>=0 && X<=99 && Y>=0 && Y<=99)
            newapple(y+1, x+1) = apple(Y+1, X+1);
        end
    end
end

% Visulize new apple

% Initialize world frame
figure(2);clf();
% imagesc(apple);
axis equal; 
xlabel('x'); ylabel('y');
axis([0, 500, 0, 500])
newapple = uint8(newapple);
imagesc(newapple);
grid on