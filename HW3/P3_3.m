% Visual Odometry Part 3: 3D to 3D motion estimation
clear
clc

% Code to call your function-----------------------------------------------
% load in the images
It0_left = rgb2gray(imread('It1_L.png'));
It0_right = rgb2gray(imread('It1_R.png'));
It1_left = rgb2gray(imread('It2_L.png'));
It1_right = rgb2gray(imread('It2_R.png'));

[R_total,t_total] = motionEstimation3Dto3D(It0_left, It0_right, It1_left, It1_right);

% Your Function------------------------------------------------------------
function [R,t] = motionEstimation3Dto3D(It0_left, It0_right, It1_left, It1_right)
    %
    % 3D to 3D motion estimation. t0 and t1 refer to the time frames t and
    % t+1, respectively.
    %
    % read calibration parameters (intrinsic and extrinsic) from the
    % calibration file
    calibname = 'calib.txt';
    T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
    B = table2array(T);
    Pleft = vertcat(B(1,1:4), B(1,5:8), B(1,9:12)); % left camera [3x4] projection matrix
    Pright = vertcat(B(2,1:4), B(2,5:8), B(2,9:12)); % right camera [3x4] projection matrix
    
    % calculate correspondences between It0_left, It0_right stereo pair
    [matchedPointsLt0, matchedPointsRt0, featureL0, ~] = detectFeatureMatches(It0_left, It0_right);
    % calculate correspondences between It1_left, It1_right stereo pair
    [matchedPointsLt1, matchedPointsRt1, featureL1, ~] = detectFeatureMatches(It1_left, It1_right); 
    
    % calculate correspondences between It0_left and It1_left (loc_ft0,loc_ft1)
    [matchedPointsLeftt0, matchedPointsLeftt1, featureLtt0, ~] = detectFeatureMatches(It0_left, It1_left);
    
    
    loc_Lt0 = double(matchedPointsLt0.Location);
    loc_Rt0 = double(matchedPointsRt0.Location);

    loc_Lt1 = double(matchedPointsLt1.Location);
    loc_Rt1 = double(matchedPointsRt1.Location);
    loc_ft0 = double(matchedPointsLeftt0.Location);
    loc_ft1 = double(matchedPointsLeftt1.Location);
    [length, ind] = min([size(loc_ft0,1), size(loc_Lt0,1), size(loc_Lt1,1)]);
    
    
    % match feature points
    f1 = featureL0;
    f2 = featureLtt0;
    f3 = featureL1;

    for i = 1:length
    f21_dist = f2 - f1(i,:);
    f21_dist = f21_dist.^2;
    f21_dist = sum(f21_dist,2);
    [~,index] = min(f21_dist);
    loc_ft0([i index],:) = loc_ft0([index, i],:);
    loc_ft1([i index],:) = loc_ft1([index, i],:);
    
    f31_dist = f3 - f1(i,:);
    f31_dist = f31_dist.^2;
    f31_dist = sum(f31_dist,2);
    [~,index] = min(f31_dist);
    loc_Lt1([i index],:) = loc_Lt1([index, i],:);
    loc_Rt1([i index],:) = loc_Rt1([index, i],:);
end

loc_ft0 = loc_ft0(1:length,:);
loc_ft1 = loc_ft1(1:length,:);

for i = 1:length
    % calculate 3D point cloud Wt0 from It0_left, It0_right stereo pair
    Wt0(i,:) = linear_triangulation(Pleft, Pright, ...
        loc_Lt0(i,:), loc_Rt0(i,:));
end

for i = 1:length
    % calculate 3D point cloud Wt1 from It1_left, It1_right stereo pair
    Wt1(i,:) = linear_triangulation(Pleft, Pright, ...
        loc_Lt1(i,:), loc_Rt1(i,:));
end

% compute R_total, t_total using ransac
[R,t] = estimateTransform(Wt0,Wt1,loc_ft0,loc_ft1, Pleft);
%EOF
end


function [R,t] = estimateTransform(Wt0,Wt1,loc_ft0,loc_ft1, Pleft)
%
%   Find the R,t between source and target point clouds (Wt0 and wt1, respectively)
%   using a set of corresponding points whose locations are given by loc_ft0,loc_ft1.
%   RANSAC method is used.
%
% OUTPUTS:
%   The estimated R and t
%
% Extract the RANSAC coefficients
coeff.numPtsToSample = 3;     % the minimum number of correspondences needed to fit a model
coeff.iterNum = 2000;         % number of iterations to run the RANSAC sampling loop
coeff.thDist = 1.5;           % inlier distance threshold; units are in pixels
coeff.source_feat_locs = loc_ft0; % 2D locations of the features in the source t0 image
coeff.target_feat_locs = loc_ft1; % 2D locations of the featuers in the target t1 image
coeff.camera_projective_mat = Pleft;   % intrinsic/projective camera matrix
coeff.randomseed = 2.8;                % use rng(2.8) to set your random seed!!!!!!!
% find those inliers!
[affT,inlierIdx] = ransac3D(Wt0, Wt1, coeff);
% find those inliers!
[R,t] = ransac3D(Wt0, Wt1, coeff);

end


function [point_cloud] = linear_triangulation(Pleft, Pright, point_left, point_right)
A = [point_left(1)*Pleft(3,:)-Pleft(1,:);
    point_left(2)*Pleft(3,:)-Pleft(2,:);
    point_right(1)*Pright(3,:)-Pright(1,:);
    point_right(2)*Pright(3,:)-Pright(2,:)];
[~,~,V] = svd(A);
point_cloud = V(:, end)'/V(end);
end


function [R,t] = ransac3D(Wt0, Wt1, coeff)
%  size(Wt0)
% size(Wt1)

num_points = size(Wt0,1);
rng(2.8)
num_inliers_last = 0;
for i = 1:coeff.iterNum
    
    indices = randperm(num_points, 3); % randomly pick 3 points
    p0 = Wt0(indices,:);
    p1 = Wt1(indices,:);
    
    p0 = p0';
    p1 = p1';
    
    min_dist = sqrt(sum((p0 - p1).^2));
    d = min_dist/std(min_dist);
    w = exp(-d)/sum(exp(-d));
    w = w';
    mu0 = p0*w;
    mu1 = p1*w;
    wi = w*ones(1,3);
    
    p0_bar = (p0(1:3,:) - mu0(1:3,:)).*wi;
    p1_bar = (p1(1:3,:) - mu1(1:3,:)).*wi;
    cov = p1_bar*p0_bar';
    [U, ~, V] = svd(cov);
    R1 = V*U';
    t1 = mu0(1:3,:) - R1*mu1(1:3,:);
    
    T = [R1 t1;
        zeros(1,3) 1];
    
    % calculate reprojection error
    jt = coeff.source_feat_locs;
    jt1 = coeff.target_feat_locs;
    %       size(jt)
    jt = jt';
    jt1 = jt1';
    
    p_temp0 = coeff.camera_projective_mat/T*Wt0';
    %     size(p_temp0)
    J0 = (jt1 - p_temp0(1:2,:)./p_temp0(3,:)).^2;
    
    p_temp1 = coeff.camera_projective_mat*T*Wt1';
    J1 = (jt - p_temp1(1:2,:)./p_temp1(3,:)).^2;
    dist = sqrt(sum(J0)) + sqrt(sum(J1));
    %     dist = sqrt(sum(error));
    %     min(dist)
    
    num_inliers = sum(dist < coeff.thDist);
    if num_inliers > num_inliers_last
        R = R1;
        t = t1;
        num_inliers_last = num_inliers;
    end
end
end