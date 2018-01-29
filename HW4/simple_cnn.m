function output = simple_cnn(vars, n_epoch, rate, p, test_intv)
%SIMPLE_CNN Performs simple CNN classification of two types of symbols
%    INPUTS
%     vars: Structure containing relevant variables
%  n_epoch: number of epochs to compute
%     rate: learning rate
%        p: pooling size/step
%test_intv: number of training epochs to pass between testing validation
%   OUTPUTS
%   output: structure containing error and classification results

% Extract inputs from struct
train_cell      = vars.train_cell;
train_labels    = vars.train_labels;
test_cell       = vars.test_cell;
test_labels     = vars.test_labels;
k_cell          = vars.k_cell;
weights         = vars.weights;

% Build arrays to store errors and number of correct classifications
train_epoch_err = zeros(n_epoch, 1);
tr_num_corr_arr = zeros(n_epoch, 1);
test_epoch_err  = zeros(floor(n_epoch/test_intv) + 1, 2);
ts_num_corr_arr = zeros(floor(n_epoch/test_intv) + 1, 2);
ts_i = 1;   % Test iterator

% Because we are only training final weights, pre-compute FCNs
train_fconv = cell(size(train_cell));
test_fconv = cell(size(test_cell));
for i = 1:numel(train_cell)
    train_fconv{i} = cnn_fwd(train_cell{i}, k_cell, p);
end
for i = 1:numel(test_cell)
    test_fconv{i} = cnn_fwd(test_cell{i}, k_cell, p);
end


% Run cnn training/testing
for i = 1:n_epoch
    % Initialize training error and correct classification vars for iteration
    train_err = zeros(numel(train_cell), 1);
    tr_num_corr = 0;
    
    % Train cnn for each training image
    for j = 1:numel(train_cell)
        % Pull training fully connected layer
        fconv = train_fconv{j};
        
        % Apply backpropagation, compute mean-squared error
        [weights, tn_err] = backprop_cnn(fconv,weights,train_labels(j,:),rate);
        train_err(i) = sum(tn_err.^2);
        
        % Check training classifications
        tr_guess = % complete ...
        tr_num_corr = tr_num_corr + eval_class(tr_guess, train_labels(j,:));
    end
    
    % Regularly compute testing performance
    if (mod(i, test_intv) == 0) || (i == 1)
        % Initialize testing error and correct classification vars
        test_err = zeros(numel(test_cell), 1);
        ts_num_corr = 0;
        
        % Evaluate accuracy with testing set (no backprop)
        for j = 1:numel(test_cell)
            % Pull testing fully connected layer
            fconv = test_fconv{j};
            
            % Compute estimate for testing image and compute mean-squared error
            ts_guess = % complete ...
            ts_err =  % complete ...
            test_err(j) = sum(ts_err.^2);
            
            % Check testing classifications
            ts_num_corr = ts_num_corr + eval_class(ts_guess, test_labels(j,:));
        end
        
        % Store testing errors/classification accuracy and epochs
        test_epoch_err(ts_i,:) = [i, mean(test_err)];
        ts_num_corr_arr(ts_i,:) = [i, ts_num_corr];
        ts_i = ts_i + 1;    % Iterate testing evaluation index
    end
    
    % Store training errors/classification accuracy
    train_epoch_err(i) = mean(train_err);
    tr_num_corr_arr(i) = tr_num_corr;
end

% Build output structure
output.train_epoch_err = train_epoch_err;
output.test_epoch_err  = test_epoch_err;
output.tr_num_corr_arr = tr_num_corr_arr;
output.ts_num_corr_arr = ts_num_corr_arr;
output.weights = weights;

% Plots
% figure()
% hold on
% plot(1:n_epoch, train_epoch_err, 'k.-')
% plot(test_epoch_err(:,1), test_epoch_err(:,2), 'bo-')
% hold off
% xlabel('Epoch')
% ylabel('Mean Sq. Error')
% legend('Training Error', 'Testing Error', 'Location', 'northeast')
% 
% tr_n = numel(train_cell);
% ts_n = numel(test_cell);
% figure()
% hold on
% plot(1:n_epoch, 100*(tr_n - tr_num_corr_arr)./tr_n, 'k.-')
% plot(ts_num_corr_arr(:,1), 100*(ts_n - ts_num_corr_arr(:,2))./ts_n, 'bo-')
% hold off
% ylim_cur = ylim;
% ylim([0, ylim_cur(2)])
% xlabel('Epoch')
% ylabel('Classification Loss (%)')
% legend('Training Loss', 'Testing Loss', 'Location', 'northeast')

end