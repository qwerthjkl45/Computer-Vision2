function [pvm, pvm_logic] = dense_chaining(set_size)
    % set size: number of frames that the algorithm should trace back to
    % find the corresponding points in current frame

    pvm = [];
    for file_idx = 1:49
        fprintf('iteration: %d\n', file_idx);    

        % put the later one into pvm
        if file_idx == 1
            % find matching points 
            img1 = read_data(1);
            img2 = read_data(2);
            [~, ~, p1, p2]=keypoint_matching(img1, img2, true);
            [F, inliers1, inliers2] = normalized_eight_point_with_RANSAC(p1, p2);
            pvm = [pvm; inliers1(1:2, :); inliers2(1:2, :)];
        else
            % need to check if there are same points in the previous frame
            pvm = [pvm; zeros(2, size(pvm, 2))];
            if file_idx < set_size
                pvm = morePVM(file_idx, pvm, 1);
            else
                pvm = morePVM(file_idx, pvm, set_size);
            end
        end

    end

    pvm_logic = pvm(1:2:100, :) > 0;
    cmap = [1 1 1 
           0 0 0];
    imagesc(pvm_logic);
    colormap(cmap);

end

function img = read_data(idx)
    file_name = sprintf('./Data/House/House/frame%08d.png', idx);    
    img = imread(file_name);
end

function [pvm] = morePVM(file_idx, pvm, set_size)

    if file_idx == 49
        img2 = read_data(1);
    else
        img2 = read_data(file_idx + 1);
    end
    
    for frame_idx = file_idx - set_size + 1 : file_idx
        img1 = read_data(file_idx);
        [~, ~, p1, p2]=keypoint_matching(img1, img2, true);
        [F, inliers1, inliers2] = normalized_eight_point_with_RANSAC(p1, p2);
        
        previous_pvm = pvm(frame_idx*2 -1: frame_idx*2, :); % 2*n
        %pvm = [pvm; zeros(2, size(pvm, 2))];
        p = 0;
        % record the points that haven't appeared in previous frame
        tmp_inliers = [];
        for idx = 1: size(inliers1, 2)
            [~, intersect_x_indices] =  intersect(previous_pvm(1, :), inliers1(1, idx));
            %n_point_match = n_point_match + size(intersect_x_indices, 1);
            [~, intersect_y_indices] =  intersect(previous_pvm(2, :), inliers1(2, idx));

            [~, tmp] = intersect(intersect_x_indices, intersect_y_indices);

            if (size(tmp, 1) >= 1)
                matched_points_index = intersect_x_indices(tmp);
                pvm(end -1: end, matched_points_index) = inliers2(1:2, idx);
            else
                % number of points that haven't apperared before
                p = p + 1;
                tmp_inliers = [tmp_inliers, idx];                                
            end

        end
        pvm = [pvm, zeros(size(pvm, 1), p)];
        pvm(frame_idx*2 -1: frame_idx*2, size(pvm, 2) -p + 1:end) = inliers1(1:2, tmp_inliers);
        pvm(end - 1: end, size(pvm, 2) - p + 1:end) = inliers2(1:2, tmp_inliers);
        
        
        
    end
    
end

