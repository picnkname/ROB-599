clear
clc

% 4.1
image = rgb2gray(imread('query.png'));
feature = SURF(image);

% 4.2
codeword = get_codeword(randn(1000, 64), 5);

% 4.3
feature_all = randn(1000, 64); % features from all images in the dataset
n_c = 5;
codeword = get_codeword(feature_all, n_c);

feature = randn(20, 64); % features from one image
kdtree = KDTreeSearcher(codeword);
h = get_hist(kdtree, feature, n_c);

% 4.4
[d, idx] = query();
plot(d)

% 4.1 Feature extraction
function feature = SURF(image)
points = detectSURFFeatures(image);
[feature, ~] = extractFeatures(image, points);
end

% 4.2 k-means clustering
function codeword = get_codeword(feature_all, n_c)
rng(0);
[~, codeword] = kmeans(feature_all, n_c);
end

% 4.3 BoW representation
function h = get_hist(kdtree, feature, n_c)
idx = knnsearch(kdtree.X, feature);
table = tabulate(idx);
h = (table(:,3)/100)';
end

% 4.4 Query
function [d, idx] = query()
kitti = load('kitti_bow.mat');
hist_train = kitti.hist_train;
kdtree = kitti.kdtree;
n_c = kitti.n_c;

img_q = rgb2gray(imread('query.png'));

feature = SURF(img_q);
hist_q = get_hist(kdtree, feature, n_c); 

d = zeros(size(hist_train, 1), 1);
for i = 1:size(hist_train, 1)
    d(i) = chi_sq_dist(hist_train(i,:), hist_q);
end

idx = find(d == (min(d)));
end

function d = chi_sq_dist(h1, h2) 
d = sum((h1 - h2).^2 ./ (h1 + h2 + eps)) / 2; 
end

