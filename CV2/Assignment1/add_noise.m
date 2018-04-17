function [point_set_noise] = add_noise(point_set, variance_parameter)
% point_set = point_set(:, 1:10);
max_val = max(max(point_set));
min_val = min(min(point_set));
max_diff = max_val - min_val;

if nargin == 1
    variance = max_diff / 100;
else
    variance = max_diff / variance_parameter;
end

mean = 0;

point_set_noise = point_set + variance*randn(size(point_set)) + mean;
% point_set_noise = imnoise(point_set, 'salt & pepper');
    
end