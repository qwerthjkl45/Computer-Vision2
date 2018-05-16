function M = sfm(pvm_with_points_coordinate, pvm_logic, size_image_set, another_decomposition, pvm_source)
    % pvm_with_points_coordinate: contains 2d projections in the form 
    % of an 2M * N  matrix, where M is the number of views and N is the 
    % number of 3D points.
    % pvm_logic: a M x N logical matrix
    % size_image_set: the size of the image set in each iteration
    % another_decomposition: different decomposition, if it's set true, the
    % orignal S will mutiple with a arbitary invertible matrix to get new S
    % pvm_source: string for figure title
    
    C_val = [];
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
        S = (W3^(0.5)) * V3';
        M = U3 * (W3^(0.5));
        if another_decomposition
            S = W3 * V3';
            M = U3;
        end
        S(3, :) = S(3, :) * 10;
        indices = find(abs(S(3, :))>20);
        S(:, indices) = [];
        
        prev_S = S;
        
        if idx > 1
            [~, S, ~] = procrustes(prev_S, S);
        else
            figure();
            scatter3(S(1, :), S(2, :), S(3, :));
            title({'3D point cloud obtained from', '1 single dense block from', pvm_source});
        end        
        
        C_val = [C_val, ones(1, size(S, 2)) * idx] ;
        merged_points = [merged_points, S];
        disp('-----')
        
    end
    figure();
    title({'3D point cloud obtained from mutiple dense blocks from', pvm_source});
    fscatter3(merged_points(1, :), merged_points(2, :), merged_points(3, :), C_val);
    
end

function newS = calcuate(M, S)
    total_frame = size(M , 1)/2;
    
    % Recover C from L by Cholesky decomposition
    % L = A\b, where A = 3m x 9, b = 3m x 1;
    A = [];
    b = [];
    for idx = 1: total_frame
        % construct A fist: ai1*L*ai1' = 1 ai2*L*ai2' = 1 ai1*L*ai2' = 0
        A1 = reshape(M(idx*2 -1, :)' * M(idx*2 -1, :), [1 9]); % ai1*L*ai1' 
        A2 = reshape(M(idx*2 , :)' * M(idx*2 , :), [1 9]); % ai2*L*ai2'  
        A1A2 = reshape(M(idx*2 -1 , :)' * M(idx*2 , :), [1 9]); % ai1*L*ai2'
        
        A = [A; A1; A2; A1A2];
        b = [b; 1; 1; 0];
   
    end
    
    L = reshape(A\b, [3,3]);
    
    % Recover C from L by Cholesky decomposition: L = CC'
    [~, P] = chol(L);
    if P == 0
        C = chol(L);
        newS = inv(C)*S;
    else
        newS = S;
    end
    %
end

