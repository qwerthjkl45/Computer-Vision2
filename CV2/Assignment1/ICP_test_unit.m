
data_source = load('./Data/source.mat');
data_target = load('./Data/target.mat');

visualize = 1;
[~, ~, ~, rms_all] =  ICP(data_source.source, data_target.target, 20);
[~, ~, ~, rms_informative] =  ICP(data_source.source, data_target.target, 20, {'informative', 1000} );
[~, ~, ~, rms_uniforms] =  ICP(data_source.source, data_target.target, 20, {'uniform', 1000} );
[~, ~, ~, rms_random] =  ICP(data_source.source, data_target.target, 20, {'random', 1000} );


%% generate graph for accuracy and speed of convergence
x = 1:1:20;
figure(1);
xlabel('iteration');
ylabel('MSE');
title('MSE and convergence rate for 1000 points from different point selection technique in each iterations');
drawnow;
hold on;
plot(x, rms_uniforms, x, rms_random, x, rms_informative, x, rms_all, 'LineWidth', 2);
legend({'uniform sub-sampling', 'random sub-sampling', 'sampling from informative regions', 'all points'});



%%  generate graph for stability
x = 1:1:10;

avg_rms_informative = zeros(10, 1);
avg_rms_uniforms = zeros(10, 1);
avg_rms_random = zeros(10, 1);
avg_rms_all = zeros(10, 1);

for idx = x
    [~, ~, avg_rms_informative(idx), ~] =  ICP(data_source.source, data_target.target, 20, {'informative', 1000} );
    [~, ~, avg_rms_uniforms(idx), ~] =  ICP(data_source.source, data_target.target, 20, {'uniform', 1000} );
    [~, ~, avg_rms_random(idx), ~] =  ICP(data_source.source, data_target.target, 20, {'random', 1000} );
    [~, ~, avg_rms_all(idx), ~] =  ICP(data_source.source, data_target.target, 20);
end

figure(2);
xlabel('number of times');
ylabel('MSE');
title('Stability for 1000 points from different point selection technique');
drawnow;
hold on;
plot(x, avg_rms_uniforms, x, avg_rms_random, x, avg_rms_informative, x, avg_rms_all, 'LineWidth', 2);
legend({'uniform sub-sampling', 'random sub-sampling', 'sampling from informative regions', 'all points'});


%%  generate graph for tolerance to noise
x = 1:1:10;

avg_rms_informative = zeros(10, 1);
avg_rms_uniforms = zeros(10, 1);
avg_rms_random = zeros(10, 1);
avg_rms_all = zeros(10, 1);

for noise_var = x
    noise_data_source = add_noise(data_source.source, noise_var);
    noise_data_target = add_noise(data_source.source, noise_var);
    
    [~, ~, avg_rms_informative(noise_var), ~] =  ICP(noise_data_source, noise_data_target, 20, {'informative', 1000});
    [~, ~, avg_rms_uniforms(noise_var), ~] =  ICP(noise_data_source, noise_data_target, 20, {'uniform', 1000} );
    [~, ~, avg_rms_random(noise_var), ~] =  ICP(noise_data_source, noise_data_target, 20, {'random', 1000} );
    [~, ~, avg_rms_all(noise_var), ~] =  ICP(noise_data_source, noise_data_target, 20);
    
end


x = 1./x;
figure(3);
xlabel('noise variance');
ylabel('MSE');
title('MSE for 1000 points from different point selection technique in difference noise variance');
drawnow;
hold on;
plot(x, avg_rms_uniforms, x, avg_rms_random, x, avg_rms_informative, x, avg_rms_all, 'LineWidth', 2);
legend({'uniform sub-sampling', 'random sub-sampling', 'sampling from informative regions', 'all points'});

%% ICP: different weight

[~, ~, ~, rms_informative] =  ICP(data_source.source, data_target.target, 20, {'informative', 1000} );
[~, ~, ~, rms_informative_improvement] =  ICP_improvement(data_source.source, data_target.target, 20, {'informative', 1000});

x = 1:1:20;
figure(4);
xlabel('iteration');
ylabel('MSE');
title('MSE and convergence rate for 1000 points from different weightening of ICP');
drawnow;
hold on;
plot(x, rms_informative, x, rms_informative_improvement, 'LineWidth', 2);
legend({'constant weight', 'linear with distance'});


