function [Gx, Gy, im_magnitude,im_direction] = compute_gradient(image)
kernel_x = [1, 0, -1;2, 0, -2; 1, 0, -1];
kernel_y = kernel_x';

Gx = imfilter(image, kernel_x);
Gy = imfilter(image, kernel_y);

im_magnitude = sqrt((Gx .^ 2) + (Gy .^ 2));

Gx = double(Gx);
Gy = double(Gy);
im_direction = atan(Gy ./ Gx);

end

