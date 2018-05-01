function best_x = RANSAC(f1, f2)
    % orig_xy size: 2 x 947
    num_matched_points = size(f1, 2);
    p = 10;
    A = zeros(2*p, 6);
    b = zeros(2*p, 1);
    best_x = zeros(6, 1);
    inliers = 0;
    
    [A_whole_img, b_whole_img] = buildab( f1(1:2,:), f2(1:2, :));
    orig_trans_xy = reshape(b_whole_img, 2, []);
    
    for n = 1: 50
        rand_array = randi([1 num_matched_points],1,p);
        for idx = 1:p
            rand_idx = rand_array(1, idx);
            A((idx*2) -1:(idx*2), :) = A_whole_img((rand_idx*2) -1:(rand_idx*2), :);
            b((idx*2) -1:(idx*2), :) = b_whole_img((rand_idx*2) -1:(rand_idx*2), :);
            
        end
        
        % calculate inliers
        x = pinv(A)*b;
        new_trans_xy = A_whole_img*x;
        new_trans_xy = reshape(new_trans_xy, 2, []);
        inliers_tmp = calculate_inliers(new_trans_xy, orig_trans_xy);
        if inliers_tmp > inliers
            best_x = x;
            inliers = inliers_tmp;
            disp('====')
            fprintf('number of inliers: %d\n', inliers);
            disp('====')
        end
    end
    
end

function inliers = calculate_inliers(new_trans_xy, orig_trans_xy)
    distance = new_trans_xy - orig_trans_xy;
    distance = sqrt(sum(distance .^ 2));
    inliers = sum(sum(distance < 10));

end

function [A, b] = buildab(orig_xy, trans_xy)
    A = zeros(2*size(orig_xy, 2), 6);
    b = zeros(2*size(orig_xy, 2), 1);
    
    for idx = 1:size(orig_xy, 2)
        x = round(orig_xy(1, idx));
        y = round(orig_xy(2, idx));
        
        A((idx*2) -1, :) = [x, y, 0, 0, 1, 0];
        A(idx * 2, :) = [0, 0, x, y, 0, 1];
        
        b((idx*2) -1, :) = round(trans_xy(1, idx));
        b(idx * 2, :) = round(trans_xy(2, idx));
    end
end

