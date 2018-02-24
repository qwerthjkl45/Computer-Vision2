function G = gauss2D( sigma , kernel_size )
    %% solution
    g = gauss1D(sigma, kernel_size);
    G = g' * g;
    G = G / sum(G);
    
end
