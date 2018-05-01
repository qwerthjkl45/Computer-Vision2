function [t_matrix, x, points_pair1, points_pair2]= keypoint_matching( orig_img1, orig_img2, stitch)

     img1 = single(orig_img1);
     img2 = single(orig_img2);
    
    if ~stitch
        stackedImage = cat(2, orig_img1, orig_img2);
    end
    
    [h, w] = size(img1);
    
    [fa, da] = vl_sift(img1);
    [fb, db] = vl_sift(img2);
    
    [matches, scores] = vl_ubcmatch(da, db, 5) ;    
    total_match_point = size(matches, 2);
    %rand_idx = randi(total_match_point, [1 total_match_point]);
    f1 = matches(1, :);
    f2 = matches(2, :);
    
    fb1 = fb;
    fb1(1, f2) = fb1(1, f2) + w;
    
    if ~stitch    
        figure(), imshow(stackedImage);
        hold on;
        h1 = vl_plotframe(fa(:,f1)) ; 
        h2 = vl_plotframe(fb1(:,f2)) ; 
        for idx = 1: size(f1,2)
            line([fa(1,f1(1, idx)) fb1(1,f2(1, idx))], [fa(2,f1(1, idx)) fb1(2,f2(1, idx))], 'Color','red');
        end
    
        hold off;
    end
    
    x = RANSAC(fa(:, f1), fb(:, f2));
    t_matrix = [x(1), x(2), 0; x(3), x(4), 0; x(5), x(6), 1];
    
    % if it is homogeneous
    points_pair1 = fa(1:2, f1);
    points_pair2 = fb(1:2, f2); 
        
    
    
end


