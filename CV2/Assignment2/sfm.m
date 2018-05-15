function merged_points = sfm(pvm_with_points_coordinate, pvm_logic, size_image_set)
    C = [];
    merged_points = [];

    for idx = 1:size(pvm_logic, 1)-size_image_set + 1
        % choose every three frames and select dense block from pvm_logic
        dense_points_indices = sum(pvm_logic(idx: idx + size_image_set -1, :)) > size_image_set -1 ;
        % construct matrix D
        D = pvm_with_points_coordinate( (idx*2)-1: (idx + size_image_set -1)*2, dense_points_indices);
         
        % Normalize the point coordinates
        D = D - mean(D, 2);
        
        % Apply SVD
        [U, W, V] = svd(D);
        U3 = U(:, 1:3);
        W3 = W(1:3, 1:3);
        V3 = V(:, 1:3);
        
        D = U3 * W3 * V3';
        M = U3 * (W3^(0.5));
        S = (W3^(0.5)) * V3';
        S(3, :) = S(3, :) * 10;
        prev_S = S;
        
        if idx > 1
            [~, S, ~] = procrustes(prev_S, S);
        else
            figure();
            scatter3(S(1, :), S(2, :), S(3, :));
        end        
        
        C = [C, ones(1, size(S, 2)) * idx] ;
        merged_points = [merged_points, S];
        disp('-----')
    end
    figure();
    fscatter3(merged_points(1, :), merged_points(2, :), merged_points(3, :), C);
    %scatter3(merged_points(1, :), merged_points(2, :), merged_points(3, :));
end

