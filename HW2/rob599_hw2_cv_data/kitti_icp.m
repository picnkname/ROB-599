%%  Load data
velodyne = load('velodyne.mat');
p = velodyne.data;
N = numel(p);

%%  Setup a 2D plot
figure(1)
clf()
scatter(p{1}(1, :), p{1}(2, :), 1, p{1}(3, :), 'b')
hold on
grid on
axis equal
axis([-80, 100, -40, 80])

%%  Relative motion
R = cell(N, 1); R{1} = eye(3);
t = cell(N, 1); t{1} = zeros(3, 1);

for k = 2:N
    [R{k}, t{k}] = icp_3d(p{k - 1}, p{k});  
end

%%  Total motion
[R_total, t_total] = get_total_transformation(R, t);

%%  Visulize the results
for k = 2:N
    fprintf('[R_total{%d}, t_total{%d}] = \n', k, k)
    disp([R_total{k}, t_total{k}])
    p2 = R_total{k}' * (p{k} - t_total{k});
    
    s2 = scatter(p2(1, :), p2(2, :), 1, 'r');
    l2 = legend({'data\{1\}', ['data\{', num2str(k), '\}']}, 'location', 'nw');
    
    saveas(gcf, num2str(k, 'hw2_kitti_icp_%02d.png'))
    
    delete(s2)
    delete(l2)
end