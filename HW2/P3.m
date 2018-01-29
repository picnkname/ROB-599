clear
clc

% 3.1.1
[p_t, p_s] = get_pnt_2d();
[match, min_dist] = match_pnt(p_t, p_s);

% 3.1.2
[R, t] = icp_2d_naive();

% 3.1.3
[R, t] = icp_2d_weighted();

% 3.2.1
velodyne = load('velodyne.mat');
p = velodyne.data;
[R, t] = icp_3d(p{1}, p{2}); 

% 3.2.2
% Pre-test code
N = 5 + randi(10);
R = cell(N, 1);
t = cell(N, 1);
for i = 1:N
[U, ~, V] = svd(randn(3, 3));
R{i} = U * diag([det(U) * det(V), 1, 1]) * V;
t{i} = randn(3, 1);
end

[R_total, t_total] = get_total_transformation(R, t);


% 3.1.1 Matching points
function [match, min_dist] = match_pnt(p_t, p_s)
% Visualize original points
plot(p_t(1,:), p_t(2,:), 'bx', p_s(1,:), p_s(2,:), 'r+');
axis equal;

% Define new vectors
n_t = size(p_t, 2); %443
n_s = size(p_s, 2); %475
match = zeros(n_s, 1);
min_dist = zeros(n_s, 1);

for i = 1:n_s
    min = 100; %a random value
    idx = 1;
    for j = 1:n_t    
        d = sqrt((p_s(1,i)-p_t(1,j))^2+(p_s(2,i)-p_t(2,j))^2);
        if (d < min)
            idx = j;
            min = d;
        end
    end
    match(i) = idx;
    min_dist(i) = min;
end
end

% 3.1.2 Naive ICP
function [R_total, t_total] = icp_2d_naive()
% Computes the total rotation and translation to align pt and ps

figure(1)
clf()
hold on
axis equal
axis([-2, 2, -2, 1])

% Get points
[p_t, p_s] = get_pnt_2d();
plot(p_t(1, :), p_t(2, :), 'bx');
xlabel('x')
ylabel('y')

k = 0;
R_total = eye(2);
t_total = zeros(2, 1);

while 1
    % Update p2, the current source cloud
    p2 = R_total' * (p_s - t_total);
    
    % Get p1, the matching points from the target cloud
    [match, min_dist] = match_pnt(p_t, p2);
    p1 = p_t(:, match);
    
    % Plot p2
    if k == 0
        l2 = plot([p1(1, :); p2(1, :)], [p1(2, :); p2(2, :)], 'k', ...
                p2(1, :), p2(2, :), 'r+');
        pause(1)
    else
        l2 = plot(p2(1, :), p2(2, :), 'r+');
    end
    
    % Centroids of p1 and p2
    mu1 = [mean(p1(1,:)); mean(p1(2,:))];
    mu2 = [mean(p2(1,:)); mean(p2(2,:))];
    
    % Center the two clouds
    p1_bar = p1 - mu1;
    p2_bar = p2 - mu2;
    
    % Estimate the rotation and translation
    [U, ~, V] = svd(p1_bar * p2_bar');
    R = V * U';
    t = -R * mu1 + mu2;
    
    % Update R_total and t_total
    t_total = inv(R_total')*t + t_total;
    R_total = R_total * R;

    % Terminate when [R, t] is very close to [I, 0]
    delta = norm([R - eye(2), t]);
    if delta > sqrt(eps)
        pause(0.02)
        delete(l2)
        k = k + 1;
        title(k)
    else
        break
    end
end

end

% 3.1.3 Weighted ICP
function [R_total, t_total] = icp_2d_weighted()
figure(1)
clf()
hold on
axis equal
axis([-2, 2, -2, 1])

% Get points
[p_t, p_s] = get_pnt_2d();
plot(p_t(1, :), p_t(2, :), 'bx');
xlabel('x')
ylabel('y')
n_t = size(p_t, 2); %443
n_s = size(p_s, 2); %475

k = 0;
R_total = eye(2);
t_total = zeros(2, 1);

while 1
    % Update p2, the current source cloud
    p2 = R_total' * (p_s - t_total);
    
    % Get p1, the matching points from the target cloud
    [match, min_dist] = match_pnt(p_t, p2);
    p1 = p_t(:, match);
    
    % Plot p2
    if k == 0
        l2 = plot([p1(1, :); p2(1, :)], [p1(2, :); p2(2, :)], 'k', ...
                p2(1, :), p2(2, :), 'r+');
        pause(1)
    else
        l2 = plot(p2(1, :), p2(2, :), 'r+');
    end
    
    % Weighted centroids of p1 and p2
    d = min_dist/std(min_dist);    
    den = 0;
    for i = 1 : n_s
        den = den + exp(-d(i));
    end
    
    w = (exp(-d))/den;
    mu1 = p1*w;
    mu2 = p2*w;
   
    % Weighted center the two clouds
    p1_bar = zeros(2, n_s);
    p2_bar = zeros(2, n_s);
    for i = 1:n_s
    	p1_bar(:,i) = (p1(:,i) - mu1) * w(i);
        p2_bar(:,i) = (p2(:,i) - mu2) * w(i);       
    end

    
    % Estimate the rotation and translation
    [U, ~, V] = svd(p1_bar * p2_bar');
    R = V * U';
    t = -R * mu1 + mu2;
    
    % Update R_total and t_total
    t_total = inv(R_total')*t + t_total;
    R_total = R_total * R;

    % Terminate when [R, t] is very close to [I, 0]
    delta = norm([R - eye(2), t]);
    if delta > sqrt(eps)
        pause(0.02)
        delete(l2)
        k = k + 1;
        title(k)
    else
        break
    end
end

end

% 3.2.1 Modify icp_2d
function [R_total, t_total] = icp_3d(p_t, p_s)
N=500;
p_t = downsample(p_t, N);
p_s = downsample(p_s, N);

figure(1)
clf()

hold on
axis equal
axis([-200, 200, -200, 200])

% Get points
plot(p_t(1, :), p_t(2, :), 'bx');
xlabel('x')
ylabel('y')

k = 0;
R_total = eye(3);
t_total = zeros(3, 1);

while 1
    % Update p2, the current source cloud
    p2 = R_total' * (p_s - t_total);
    
    % Get p1, the matching points from the target cloud
    [match, min_dist] = match_pnt_v2(p_t, p2);
    p1 = p_t(:, match);
    
    % Plot p2
    if k == 0
        l2 = plot([p1(1, :); p2(1, :)], [p1(2, :); p2(2, :)], 'k', ...
                p2(1, :), p2(2, :), 'r+');
        pause(1)
    else
        l2 = plot(p2(1, :), p2(2, :), 'r+');
    end
    
    % Centroids of p1 and p2
    for i=1:length(p1)
        d(i)=min_dist(i)/std(min_dist);
        e(i)=exp(-d(i));
    end

    for i=1:length(p1)
        w(i)=e(i)/sum(e);
    end
    
    mu1=p1*w';
    mu2=p2*w';

    for i=1:length(p1)   
        p1_bar(:,i) = (p1(:,i) - mu1)*w(i);
        p2_bar(:,i) = (p2(:,i) - mu2)*w(i);
    end
    
    % Estimate the rotation and translation
    [U, ~, V] = svd(p1_bar * p2_bar');
    R = V*U';
    t = mu2-R*mu1;

    % Update R_total and t_total
    t_total = inv(R_total')*t + t_total;
    R_total = R_total*R;

    % Terminate when [R, t] is very close to [I, 0]
    delta = norm([R - eye(3), t]);
    if delta > sqrt(eps)
        pause(0.02)
        delete(l2)
        k = k + 1;
        title(k)
    else
        break
    end
    
end
end

function [match, min_dist] = match_pnt_v2(p_t, p_s)
  ns=length(p_s);
  nt=length(p_t);
  for i=1:ns
      for j=1:nt
           d(i,j)=norm(p_s(:,i)-p_t(:,j));
      end
      [min_dist(i),match(i)]=min(d(i,:));
  end
  min_dist = min_dist';
  match = match';
end

function p_ = downsample(p,N)
        idx = int64(linspace(1,size(p,2),N));
        p_ = p(:,idx);
end


% 3.2.2
function [R_total, t_total] = get_total_transformation(R, t)
    R_total = {size(R,1)};
    t_total = {size(t,1)};
    R_total{1} = R{1};
    t_total{1} = t{1};
  
  for i=2:size(R,1)
      R_total{i,1} = R{i,1} * R_total{i-1,1};
      t_total{i,1} = inv(R{i,1}') * t_total{i-1,1} + t{i,1};
  end
end