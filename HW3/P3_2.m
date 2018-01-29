% Visual Odometry Part 2: generate 3D point cloud from stereo image pair
clear
clc

% Code to call your function----------------------------------------------
% load in images
Ileft = rgb2gray(imread('It1_L.png'));
Iright = rgb2gray(imread('It1_R.png'));
% load in calibration data
calibname = 'calib.txt';
T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
B = table2array(T);
Pleft = vertcat(B(1,1:4), B(1,5:8), B(1,9:12)); % left camera [3x4] projection matrix
Pright = vertcat(B(2,1:4), B(2,5:8), B(2,9:12)); % right camera [3x4] projection matrix
% call function!
[point_cloud] = gen_pointcloud(Ileft, Iright, Pleft, Pright);


% Your Function-----------------------------------------------------------
function point_cloud = gen_pointcloud(Ileft, Iright, Pleft, Pright)
%
% performs image triangulation on stereo pair given by Ileft and Iright

%
% extract and match SURF features from the left and right images
[matchedPointsL, matchedPointsR] = detectFeatureMatches(Ileft, Iright); 

%For all point correspondences
for i = 1:size(matchedPointsL,1)
    % For all of the matched/corresponding points between ImageL and ImageR
    %
    % Perform linear triangulation
    % YOU NEED TO IMPLEMENT THIS FUNCTION
    % point_cloud = linear_triangulation(point_left, point_right);
    
    x = matchedPointsL.Location(i,1);
    y = matchedPointsL.Location(i,2);
    x_prime = matchedPointsR.Location(i,1);
    y_prime = matchedPointsR.Location(i,2);
    A = [x * Pleft(3,:) - Pleft(1,:);
         y * Pleft(3,:) - Pleft(2,:);
         x_prime * Pright(3,:) - Pright(1,:);
         y_prime * Pright(3,:) - Pright(2,:)];
     
    [U, ~, V] = svd(A);
    point_cloud(i,1:3) = double(V(1:3,4)/V(4,4));
    
end

%
% Visualization Code
%scatter3(point_cloud(:,1), point_cloud(:,2), point_cloud(:,3))

end