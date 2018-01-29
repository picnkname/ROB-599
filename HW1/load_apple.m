function apple = load_apple(file_path)
% load in the image
apple = imread(file_path);
% convert to grayscale, clip, flip image horizontally
apple = double(rgb2gray(apple));
%EOF
end