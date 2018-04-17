function [rotation, translation] = ICP_sampling(base_point_set, target_point_set, iteration, sampling, visualize)
    
    % for visualize
    orig_base_point_set = base_point_set;
    orig_target_point_set = target_point_set;
    
    % initlize R and t
    rotation = eye(3);
    translation = zeros(3, 1);
    
    sampling_way = sampling{1};
    sampling_num = sampling{2};
    
    % random indices for uniform sampling
    random_indices = randperm(size(orig_base_point_set, 2), sampling_num);
        
    for iter = 1:iteration
      
        if sampling_way == "random"
            base_point_set = (datasample(orig_base_point_set', sampling_num))';   
            target_point_set = (datasample(orig_target_point_set', sampling_num))'; 
        elseif sampling_way == "uniform"
            base_point_set = orig_base_point_set(:, random_indices);
            target_point_set = orig_target_point_set(:, random_indices);
%         elseif sampling_way == "informative" 
        end
        
        num_points = size(base_point_set, 2);
        
        % find the closet point for each point in the base point set from the
        % target set point
        distances = pdist2( base_point_set', target_point_set');
        [min_dist, target_set_matched_indices] = min(distances, [], 2);
        target_point_set1 = target_point_set(:, target_set_matched_indices);
        rms = mean(min_dist);
        fprintf('rms: %5f\n', rms); 
        

        % calcualte svd
        W = eye(num_points);
        p = mean(base_point_set');
        q = mean(target_point_set1');
        S = ((base_point_set - p') * W)*(target_point_set1-q')'; %(3xn)x(nxn)x(nx3)
        [U, ~, V] = svd(S);

        % calculate R and t
        W = eye(3);
        W(3, 3) = det(V*U');
        R = V* W* U';
        t = q' - (R*p');

        orig_base_point_set = (R* orig_base_point_set)+t;
        base_point_set = (R* base_point_set)+t;
        
        % update rotation nad translation
        rotation = R * rotation;
        translation = R * translation + t;   
        
        % visualize part 1 data
        if nargin == 5 && visualize == 1
            plot3(orig_target_point_set(1, :), orig_target_point_set(2, :), orig_target_point_set(3, :), 'ro', ...
                orig_base_point_set(1, :), orig_base_point_set(2, :), orig_base_point_set(3, :), 'bo');
            pause(1)
        end
        
    end
                                                                                
end
