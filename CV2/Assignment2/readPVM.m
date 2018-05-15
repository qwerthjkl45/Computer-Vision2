function [pvm, pvm_logic]= readPVM
    fullFileName = fullfile(pwd, 'PointViewMatrix.txt');
    pvm = readtable(fullFileName);
    
    % convert to matrix
    pvm = pvm{:, :};
    pvm_logic = pvm(1:2:202, :)>0;
    cmap = [1 1 1 
           0 0 0];
    imagesc(pvm_logic);
    colormap(cmap);
end