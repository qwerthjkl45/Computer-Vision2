function merge1()
 
    sample_size = 2000;
    C = ones(sample_size, 1);
    
    first = true;
    visualize_data = 0;
    
    step_size = 1;
    image_indices = 1:step_size:98;
    
    for idx = image_indices
        camera_points_source = Pcd_data(idx);
        camera_points_target = Pcd_data(idx + 1);

        C = ones(sample_size, 1);

        % random sample
        camera_points_source = (datasample(camera_points_source, sample_size))';
        camera_points_target = (datasample(camera_points_target, sample_size))';
    
        [rotation, translation] = ICP(camera_points_source, camera_points_target, 20);
        camera_points_source = rotation*camera_points_source + translation;

        C = [C;ones(sample_size, 1)*(idx+1)];
        if first 
            visualize_data = camera_points_target;
            first = false;
        else
            visualize_data = [camera_points_target, camera_points_source];
        end
        
    end
    
    fscatter3(visualize_data(1, :), visualize_data(2, :), visualize_data(3, :), C');
end


function points = Pcd_data(idx)
    file_name = sprintf('./Data/data/%010d.pcd', idx);
    points = readPcd(file_name);
    points = points(:, 1:3);
    
    % only extract points whose z < 2, otherwise too many moise
    points = points(points(:, 3) < 2, :);
end