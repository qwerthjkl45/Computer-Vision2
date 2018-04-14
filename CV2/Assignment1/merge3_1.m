function merge3_1()
 
    sample_size = 2000;
    visualize_data = (Pcd_data(1)) ;
    %for cmap
    C = ones(size(visualize_data, 1), 1);
    
    step_size = 3;
    start_img_idx = 1;
    end_img_idx = 97;
    image_indices = start_img_idx:step_size:end_img_idx;
    
    for idx = image_indices
        disp(idx)
        camera_points_source = Pcd_data(idx);
        
        if (idx + step_size) > end_img_idx
            break
        end
        
        camera_points_target = Pcd_data(idx+step_size);

        % random sample
        camera_points_source = datasample(camera_points_source, sample_size, 1);
        camera_points_target = datasample(camera_points_target, sample_size, 1);
    
        [rotation, translation] = ICP(camera_points_source', camera_points_target', 20);
        camera_points_source = (rotation*camera_points_source' + translation)';

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