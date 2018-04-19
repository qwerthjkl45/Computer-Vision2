function [data] = informative_region(input_data)

        [u, s, v] = svd(input_data, 'econ' );
        data = u(:, 1:2)*s(1:2, 1:2)*v(:, 1:2)';
        
end