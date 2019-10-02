addpath /usr/local/MATLAB/R2018b/toolbox/vision/TOOLBOX_calib

% read image and convert it into grayscale
% D = './calib_phone/';
D = './Latino_SC_50%_x5/Calibrated/';
S = dir([D 'Image__rect*.bmp']); % pattern to match filenames.
I = cell(numel(S),1);
for k = 1:numel(S)
    F = fullfile(D,S(k).name);
    % I{k} = rgb2gray(imread(F));
    I{k} = imread(F);
    file_name = [D 'Image_rect_' num2str(k) '.jpg'];
    imwrite(I{k}, file_name);
end