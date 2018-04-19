function merge3_1()
 
    sample_size = 2000;
    step_size = 1;
    start_img_idx = 1;
    end_img_idx = 99;
    image_indices = start_img_idx:step_size:end_img_idx;
    
    visualize_data = datasample((Pcd_data(1)), sample_size, 1); 
    %for cmap
    C = ones(size(visualize_data, 1), 1);
    
    R = zeros(3, 3);
    T = zeros(3, 1);
    
    for idx = image_indices
        if ((idx + step_size) > end_img_idx)
            break
        end
        camera_points_source = Pcd_data(idx);        
        camera_points_target = Pcd_data(idx+step_size);

        % random sample
        camera_points_source = datasample(camera_points_source, sample_size, 1);
        camera_points_target = datasample(camera_points_target, sample_size, 1);
    
        [rotation, translation, rms, ~] = ICP(camera_points_source', camera_points_target', 20);
        fprintf('=== frame: %d, rms: %f ===\n', idx, rms);
        
        % update R and T
        if all(R == 0)
            R = rotation;
            T = translation;
        else
            R =  R * rotation;
            T = rotation * T + translation;   
        end        
        
        camera_points_source = (R*camera_points_source' + T)';          
        
        C = [C; ones(sample_size, 1) * (idx+1)];
        visualize_data = [visualize_data; camera_points_source];
        
    end
    
    fscatter3(visualize_data(:, 1), visualize_data(:, 2), visualize_data(:, 3), C);
end


function points = Pcd_data(idx)
    file_name = sprintf('./Data/data/%010d.pcd', idx);
    points = readPcd(file_name);
    points = points(:, 1:3);
    
    % only extract points whose z < 2, otherwise too many moise
    points = points(points(:, 3) < 2, :);
end