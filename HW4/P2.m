% Problem 2: Simple Neural Network
clear
clc

vars = load('simple_nn_vars.mat');
n_epoch = 2000;
rate = [0.04, 0.2];
test_intv = 100;

output_nn = simple_nn(vars, n_epoch, rate, test_intv);

% --------------------------------Your Function----------------------------

function output = simple_nn(vars, n_epoch, rate, test_intv)
%SIMPLE_NN Performs simple neural network estimation of linear model
%    INPUTS
%     vars: Structure containing relevant variables
%  n_epoch: number of epochs to compute
%     rate: learning rate
%test_intv: number of training epochs to pass between testing validation
%   OUTPUTS
%   output: structure containing error and classification results

% Extract variables
system_len  = vars.system_len;
test_in     = vars.test_in;
test_out    = vars.test_out;
train_in    = vars.train_in;
train_out   = vars.train_out;

% Initialize random seed
rng(0);

% Initialize synapses
syn0 = 2.*rand(system_len, 4) - 1;
syn1 = 2.*rand(4, 1) - 1;

% Initialize error arrays and test iterator
l2_err_train = zeros(n_epoch, 1);
l2_err_test = zeros(floor(n_epoch/test_intv) + 1, 2);
ts_i = 1;

% Run neural network training/testing for given number of epochs
for i = 1:n_epoch
    % Run training forward propagation
    [l0, l1, l2, l2_err] = simple_nn_fwd(train_in, train_out, syn0, syn1);
    
    % Compute mean squared testing error for final layer
    l2_err_train(i) = mean(l2_err.^2);
    
    % Regularly get nnet accuracy for test data
    if (mod(i, test_intv) == 0) || (i == 1)
        % Run testing forward propagation
        [~, ~, ~, l2_ts_err] = simple_nn_fwd(test_in, test_out, syn0, syn1);
        
        % Compute mean squared error for final layer and iterate index
        l2_err_test(ts_i,:) = [i, mean(l2_ts_err.^2)];
        ts_i = ts_i + 1;
    end
    
    % Perform backpropagation
    [syn0, syn1] = backprop_simple(l0, l1, l2, l2_err, syn0, syn1, rate);
end

% Build output structure
output.syn0 = syn0;
output.syn1 = syn1;
output.l2_err_train = l2_err_train;
output.l2_err_test = l2_err_test;

% Plot (optianal)
figure()
hold on
plot(1:n_epoch, l2_err_train, 'k.-')
plot(l2_err_test(:,1), l2_err_test(:,2), 'bo-')
hold off
xlabel('Epoch')
ylabel('Mean Sq. Error')
legend('Training Error', 'Testing Error', 'Location', 'northeast')

end


%------------------------------------------------------------------------------%
%---------------------------Add Helper Functions Here--------------------------%
%------------------------------------------------------------------------------%
function output = Sigmoid(input, flag)
    output = zeros(size(input,1), size(input,2));

    for i = 1:size(input,1)
        for j = 1:size(input,2)
            if (flag == 0)
                output(i,j) = 1/(1+exp(-input(i,j)));
            elseif(flag == 1)
                output(i,j) = (1 - input(i,j)) .* input(i,j);
            end
        end
    end
end


function [l0, l1, l2, l2_err] = simple_nn_fwd(data_in, data_out, syn0, syn1)
    l0 = data_in;
    l1 = Sigmoid(l0 * syn0, 0);
    l2 = Sigmoid(l1 * syn1, 0);
    l2_err = data_out - l2;
end


function [w0, w1] = backprop_simple(l0, l1, l2, l2_err, syn0, syn1, r)
    delta_l2 = l2_err .* Sigmoid(l2, 1);
    l1_err = delta_l2 * syn1';
    
    delta_l1 = l1_err .* Sigmoid(l1, 1);
    
    w0 = syn0 + r(1) * l0' * delta_l1;
    w1 = syn1 + r(2) * l1' * delta_l2;

end
