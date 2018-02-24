function [ imOut ] = denoise( image, kernel_type, varargin)

[h, w] = size(image);
tmp = zeros(h, w, 3, 'uint8');

switch kernel_type
    case 'box'
        for i = 1:size(varargin, 2)
            imOut = imboxfilt(image, varargin{i});
            tmp(:, :, i) = imOut;
        end
        
    case 'median'
        for i = 1:size(varargin, 2)
            imOut = medfilt2(image, [varargin{i} varargin{i}]);
            tmp(:, :, i) = imOut;
        end
    case 'gaussian'
        for i = 1:size(varargin, 2)
            filter = gauss2D(varargin{i}, 7);
            imOut = imfilter(image, filter);
            tmp(:, :, i) = imOut;
        end
end

if not (all(tmp(:) == 0))
    subplot(2, 2, 1);
    imshow(image);
    title('original image');

    subplot(2, 2, 2);
    imshow(tmp(:, :, 1));
    psnr_val = myPSNR(image, tmp(:, :, 1));
    str = "";
    if contains(kernel_type, 'gaussian')
        str = strcat(kernel_type, ' with sigma 0.5. PSNR val: ');
    else
        str= strcat(kernel_type, ' with size 3x3. PSNR val: ');
    end
    str = strcat(str, num2str(psnr_val));
    title(str);

    subplot(2, 2, 3);
    imshow(tmp(:, :, 2));
    psnr_val = myPSNR(image, tmp(:, :, 2));
    str = "";
    if contains(kernel_type, 'gaussian')
        str = strcat(kernel_type, ' with sigma 1. PSNR val: ');
    else
        str= strcat(kernel_type, ' with size 5x5. PSNR val: ');
    end
    str = strcat(str, num2str(psnr_val));
    title(str);

    subplot(2, 2, 4);
    imshow(tmp(:, :, 3));
    psnr_val = myPSNR(image, tmp(:, :, 3));
    str = "";
    if contains(kernel_type, 'gaussian')
        str = strcat(kernel_type, ' with sigma 2. PSNR val: ');
    else
        str= strcat(kernel_type, ' with size 7x7. PSNR val: ');
    end
    str = strcat(str, num2str(psnr_val));
    title(str);    
    
else
    imshow(imOut);    
end

end
