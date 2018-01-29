% Visual Odometry Part 1: Compute Disparity Map
clear;
clc
% load in images
Il = rgb2gray(imread('It1_L.png'));
Ir = rgb2gray(imread('It1_R.png'));
min_d=0;
max_d=64;
w_radius= 3.0; % half of the window size
[D] = genDisparityMap(Il, Ir, min_d, max_d, w_radius);

% -----------------------copy starts--------------------------------------
function [D] = genDisparityMap(I1, I2, min_d, max_d, w_radius)
%
% INPUTS:
% I1 = left stereo image
% I2 = right stereo image
% maxdisp = determines the region of interested to search in the right image.
%           It can be thought of as the 'shift' of the right image
% w_radius = half of window size
%
ws = 2*w_radius+1;

I1 = double(I1);
I2 = double(I2);
c = size(I1, 2);
r = size(I1, 1);
% PSEUDO CODE:
% for each pixel (i,j) in the left image:
%     %for each shift value d of the right image:
%     for d = 0:maxdisp
%         % get the template using ws from the left image
%         % compare to the right image, which is shifted by d
%         % compute the Sum of absolute difference, which is a scalar
%         disp_img(i, j, d+1) = SAD
%     end
% end

for d = min_d:max_d
    I2_temp = [zeros(r, d), I2];
    I_SAD = abs(I1 - I2_temp(1:r, 1:c));
    I_SAD = imfilter(I_SAD, ones(ws, ws)/(49));
    for i = 1: r
        for j = 1: c
            SAD = I_SAD(i, j);
            disp_img(i, j, d+1) = SAD;
        end
    end
end

% because of our chosen similarity metric, we want to minimize SAD values:
%D = %? FILL IN based upon the struture of disp_image
[~,D] = min(disp_img,[],3);

% %visualize disparity map
% figure;
% imagesc([0 max_d]);
% colormap(gray);
% colorbar;
% axis image;

end
