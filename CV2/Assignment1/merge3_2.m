function merge3_2()
     
    sample_size = 2000;
    step_size = 1;
    start_img_idx = 1;
    end_img_idx = 99;
    
    image_indices = start_img_idx:step_size:end_img_idx;
    
    camera_points_target = (Pcd_data(1));
    % merged points
    visualize_data =  datasample(camera_points_target, sample_size, 1); 
    % for cmap visualize
    C = ones(size(visualize_data, 1), 1);
    
    for idx = image_indices
        if ((idx + step_size) > end_img_idx)
            break
        end
        camera_points_source = Pcd_data(idx); % n*3

        % random sample
        camera_points_source_sample = datasample(camera_points_source, sample_size, 1); % n*3
        camera_points_target_sample = datasample(camera_points_target, sample_size, 1); % n*3
      
    
        [rotation, translation, rms, ~] = ICP(camera_points_source_sample', camera_points_target_sample', 50);
        fprintf('=== frame: %d, rms: %f ===\n', idx, rms);
        camera_points_target = ((inv(rotation))*(camera_points_target' - translation))';
        
        % merge points after transformation of points in new frame
        camera_points_target = [camera_points_target; camera_points_source];

        visualize_data =  ((inv(rotation))*(visualize_data' - translation))';
        visualize_data = [visualize_data; camera_points_source_sample];
        C = [C; ones(sample_size, 1) * (idx)];
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


