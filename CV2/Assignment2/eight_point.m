function F = eight_point(points_pair1, points_pair2)


    % only extract random 9 pairs since we want F is singular.
    num_points = 9;
    random_indices = randi(size(points_pair1, 2), 1, num_points);
    points_pair1 = points_pair1(: , random_indices);
    points_pair2 = points_pair2(: , random_indices);
    
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
    % check if rank of F is 2
    rank(F)
    
end