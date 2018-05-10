% % run('~/Downloads/vlfeat-0.9.21/toolbox/vl_setup');


data1 = './Data/House/House/frame00000001.png';
data2 = './Data/House/House/frame00000040.png';
%data3 = './Data/House/House/boat1.pgm';


img1 = imread(data1);
img2 = imread(data2);

%keypoint_matching(img1, img2, false);
[~, ~, p1, p2]=keypoint_matching(img1, img2, true);


%% eight point w/o normalization
f = eight_point(p1, p2);
p1s = [p1; ones(1, size(p1, 2))];
p2s = [p2; ones(1, size(p2, 2))];
epipolar_plane = f' * p2s;
epipolar_lines(epipolar_plane, p1s, img1);

%% eight ponts with normalization
f = eight_point(p1, p2);
epipolar_plane = f' * p2s;
epipolar_lines(epipolar_plane, p1s, img1);
epipolar_plane2 = f * p1s; 
epipolar_lines(epipolar_plane2, p2s, img2);

%% normalized_eight_point_with_RANSAC
[F, inliers1, inliers2] = normalized_eight_point_with_RANSAC(p1, p2);
epipolar_plane = F' * inliers2; 
epipolar_lines(epipolar_plane, inliers1, img1);
epipolar_plane2 = F * inliers1; 
epipolar_lines(epipolar_plane2, inliers2, img2);

%% part 2: chaining
[pvm, pvm_logic] = chaining;

%% 
sfm(pvm, pvm_logic, 3);