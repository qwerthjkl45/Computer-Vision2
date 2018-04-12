data_source = load('./Data/source.mat');
data_target = load('./Data/target.mat');



visualize = 1
[R, t] =  ICP(data_source.source, data_target.target, 20, {'uniform', 1000}, visualize );

