function F = normalized_eight_point(points_pair1, points_pair2)

    % only extract random 9 pairs since we want F is singular.
    num_points = 9;
    random_indices = randi(size(points_pair1, 2), 1, num_points);
    points_pair1 = points_pair1(: , random_indices);
    points_pair2 = points_pair2(: , random_indices);
    
    % Normalization
    [points_pair1, T1] = Normalization(points_pair1);
    [points_pair2, T2] = Normalization(points_pair2);   
    
    
    % construct the A: nx9 matrix
    A_tmp = [points_pair2', ones(num_points, 1)];
    A = (points_pair1(1, :)' .* A_tmp);
    A = [A, (points_pair1(2, :)' .* A_tmp)];
    A = [A, A_tmp];
    
    % find the SVD of A
    [~,~,V] = svd(A);
    F = V(:, end);
    % let F has singular property and of rank two
    F = reshape(F, 3, 3);
    [U_f,D_f,V_f] = svd(F);
    D_f(end, end) = 0;
    
    % recompute F
    F = U_f* D_f* V_f';
    
    % Denormalization
    F = T2'*F*T1;
    
    
end


function [normalized_points, T] = Normalization(input_points)
    total_points = size(input_points, 2);
    m = mean(input_points, 2);
    d = 0;
    d1 = 0;
    
    for idx = 1: total_points
        d = d + pdist([input_points(:,idx)'; m'], 'euclidean');
    end
    
    d = d / total_points;
    % construct T
    d = sqrt(2)/d;
    T = [d, 0, -m(1)*d; 0, d, -m(2)*d; 0, 0, 1];
    
    input_points = [input_points; ones(1, total_points)];
    normalized_points = T * input_points;
    normalized_points = normalized_points(1:2, :);   
    
    
end