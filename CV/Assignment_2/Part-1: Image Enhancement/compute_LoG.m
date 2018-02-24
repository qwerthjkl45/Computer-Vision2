function imOut = compute_LoG(image, LOG_type)

switch LOG_type
    case 1
        kernel = fspecial('gaussian', 5, 0.5);
        smoothed_img = imfilter(image, kernel);
        kernel = fspecial('laplacian');
        imOut = imfilter(smoothed_img, kernel);
    case 2
        kernel = fspecial('log', 5, 0.5);
        imOut = imfilter(image, kernel);
    case 3
        kernel1 = fspecial('gaussian', 5, 0.9);
        kernel2 = fspecial('gaussian', 5, 0.1);
        img1 = imfilter(image, kernel1);
        img2 = imfilter(image, kernel2);
        imOut = img1 - img2;

end
end

