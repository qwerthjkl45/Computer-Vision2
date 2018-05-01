% % run('~/Downloads/vlfeat-0.9.21/toolbox/vl_setup');


data1 = './Data/House/House/frame00000001.png';
data2 = './Data/House/House/frame00000002.png';
%data3 = './Data/House/House/boat1.pgm';


img1 = imread(data1);
img2 = imread(data2);


%imshow(img2);
%keypoint_matching(img1, img2, false);
[~, ~, p1, p2]=keypoint_matching(img2, img1, true);
[F, inliers1, inliers2] = normalized_eight_point_with_RANSAC(p1, p2);
epipolar_plane = F' * inliers2; 
epipolar_lines(epipolar_plane, inliers1, img1);