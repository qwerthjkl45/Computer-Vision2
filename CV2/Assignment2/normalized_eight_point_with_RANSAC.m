function[F, inliers1, inliers2] = normalized_eight_point_with_RANSAC(points_pair1, points_pair2)

    total_points = size(points_pair1, 2);
    iteration = 100;
    threshold = 1;
    max_num_inliers = 0;
    F = 0;
    inliers1 = [];
    inliers2 = [];
    
    p1 = [points_pair1; ones(1, total_points)]; % 3xn
    p2 = [points_pair2; ones(1, total_points)];
    
    for idx = 1: iteration
        num_inliers = 0;
        inliers1_tmp = [];
        inliers2_tmp = [];
        F_tmp = normalized_eight_point(p1(1:2, :), p2(1:2, :));
        
        % calculate inliers:
        % Sampson distance:
        for i = 1: total_points
           d =  (p2(:, i)'* F_tmp * p1(:, i)) ^ 2;
           d_tmp = F_tmp * p1(:, i);
           d_tmp1 = F_tmp' * p2(:, i);
           d = d/((sum(d_tmp(1:2, 1).^2)) + (sum(d_tmp1(1:2, 1).^2)));
           
           if d <= threshold
               num_inliers = num_inliers + 1;
               inliers1_tmp = [inliers1_tmp, p1(:, i)];
               inliers2_tmp = [inliers2_tmp, p2(:, i)];
           end
        end  
        
        if max_num_inliers < num_inliers
            F = F_tmp;
            max_num_inliers = num_inliers;
            inliers1 = inliers1_tmp;
            inliers2 = inliers2_tmp;
            size(inliers1_tmp)
            disp(num_inliers);
            disp('--------');
        end
        
    end
    
       
    
end


