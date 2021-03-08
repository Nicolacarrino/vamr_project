clear all;
close all;
clc;
%% Setup
ds = 0;  % 0: KITTI, 1: Malaga, 2: parking, 3:Recording Dataset(our dataset)
num_last_frames_plot = 30; % frames to plot
BA_flag = true;   % use or not use BA        

% settings
kitti_path = '../kitti';
malaga_path = '../malaga-urban-dataset-extract-07';
parking_path = '../parking';
recording_path = 'dataset/recording';

addpath('./bundleAdjustment');
addpath('./keypoint');
addpath('./localization');
addpath('./localization/plot');
addpath('./parameters')
addpath('./plotting')
addpath('./pose');
addpath('./pose/8point');
addpath('./pose/plot');
addpath('./pose/triangulation');
addpath('./poseRefinement');
addpath('./projection');

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    
    [initializationParam, continuousParam] = KittiParameters(K);
    
    bootstrap_frames = [1, 5];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    
    [initializationParam, continuousParam] = MalagaParameters(K);
    
    bootstrap_frames = [1, 3];
    
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    
    [initializationParam, continuousParam] = ParkingParameters(K);
    
    bootstrap_frames = [20, 23];
elseif ds == 3
    % Path containing images, depths and all...
    assert(exist('recording_path', 'var') ~= 0);
    last_frame = 1000;
 
    K = [1895.3,      0,    962;
              0, 1895.3,  515.8;
              0,      0,      1];
          
    [initializationParam, continuousParam] = RecordingParameters(K);

    bootstrap_frames = [1, 3];
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 3
    img0 = imread([recording_path ...
        sprintf('/image_%06d.png',bootstrap_frames(1))]);
    img1 = imread([recording_path ...
        sprintf('/image_%06d.png',bootstrap_frames(2))]);
else
    assert(false);
end

%% Initialization
% Establish keypoint correspondences between these two frames 

% % select keypoints
disp('Initialization on bootstraps frames')
keypoints_0 = detectHarrisFeatures(img0, 'MinQuality', initializationParam.MinQuality, 'FilterSize', initializationParam.FilterSize);
keypoints_0 = keypoints_0.Location;

pointTracker_kepoints = vision.PointTracker('NumPyramidLevels', initializationParam.NumPyramidLevels, ...
   'MaxBidirectionalError', initializationParam.MaxBidirectionalError, 'BlockSize', initializationParam.BlockSize, ...
   'MaxIterations', initializationParam.MaxIterations);
initialize(pointTracker_kepoints,keypoints_0,img0) 
[points,point_validity] = pointTracker_kepoints(img1);

matched_keypoint_0 = keypoints_0(point_validity, :)';
matched_keypoint_1 = points(point_validity, :)';

% plot
figure(1);
imshow(img0);
hold on;
plot(matched_keypoint_0(1, :), matched_keypoint_0(2, :), 'rx', 'Linewidth', 1);

figure(2);
imshow(img1);
hold on;
plot(matched_keypoint_1(1, :), matched_keypoint_1(2, :), 'rx', 'Linewidth', 1);
plot([matched_keypoint_0(1, :); matched_keypoint_1(1, :)],...
     [matched_keypoint_0(2, :); matched_keypoint_1(2, :)], 'g-', 'Linewidth', 1);


[F_Ransac,inliersIndex] = estimateFundamentalMatrix(matched_keypoint_0',...
    matched_keypoint_1', 'Method', initializationParam.Method, ...
    'NumTrials', initializationParam.NumTrials, 'DistanceThreshold', initializationParam.DistanceThreshold);

matched_keypoint_0 = matched_keypoint_0(:,inliersIndex);
matched_keypoint_1 = matched_keypoint_1(:,inliersIndex);

[Orientation, Location] = relativeCameraPose(F_Ransac, initializationParam.Intrinsics, matched_keypoint_0', matched_keypoint_1');
[R_C_W, t_C_W] = cameraPoseToExtrinsics(Orientation, Location);

camMatrix1 = cameraMatrix(initializationParam.Intrinsics, eye(3), zeros(1,3));
camMatrix2 = cameraMatrix(initializationParam.Intrinsics, R_C_W, t_C_W);

landmark = triangulate(matched_keypoint_0', matched_keypoint_1', camMatrix1, camMatrix2)';
landmark_camera = R_C_W'*landmark + t_C_W'; 

% remove clearly oulier landmarks
median_dim = median(landmark_camera, 2);
idx = landmark_camera(3,:)>0;
idx = idx & landmark_camera(3,:) < initializationParam.RangeDepth*median_dim(3);

landmark = landmark(:,idx);
matched_keypoint_0 = matched_keypoint_0(:,idx);
matched_keypoint_1 = matched_keypoint_1(:,idx);

pose = [R_C_W t_C_W'];

% 3D plot
figure(90)
plot3(landmark(1,:), landmark(2,:), landmark(3,:), 'o');
hold on;
plotCoordinateFrame(eye(3),zeros(3,1), 1);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R_C_W*t_C_W';
plotCoordinateFrame(R_C_W,center_cam2_W, 1);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
axis equal
rotate3d on;
% end 3D plot

S = cell(last_frame,6);
S_BA = cell(last_frame,6);

S{1, 1} = matched_keypoint_0; % keypoints associated with landmarks
S{1, 2} = landmark; %landmarks
S{1, 3} = []; % candidate keypoints
S{1, 4} = []; % first observations of candidate keypoints
S{1, 5} = []; % first pose of candidate keypoints
S{1, 6} = [eye(3) zeros(3,1)];  % also keep track of camera pose in the image frame

kpt_image = detectHarrisFeatures(img1, 'MinQuality', initializationParam.MinQuality, 'FilterSize', initializationParam.FilterSize);
keypoints_image = kpt_image.Location;

dist_p = pdist2(matched_keypoint_1',keypoints_image,'squaredeuclidean', 'Smallest', 1);
new_candidate_keypoints = keypoints_image(dist_p > initializationParam.Threshold_same_point,:);

% State for the second bootstrap frame
S{2, 1} = matched_keypoint_1;
S{2, 2} = landmark;
S{2, 3} = new_candidate_keypoints';
S{2, 4} = new_candidate_keypoints';
S{2, 5} = repmat(reshape(pose,[],1),1,size(new_candidate_keypoints,1));
S{2, 6} = pose;

% if continuousParam.poseRefinementFlag
%     S(2,:) = poseRefinement(S(2,:), K);
% end

% S(1:2,:) = BundleAdjustment(S(1:2,:), K);

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;

I_prev = img1;
path = extractPath(S(1:2,:));
last_keyframe_pos = [0 0 0]';
last_keyframe_idx = 1;
before_last_keyframe_idx = 1;

num_tracked_landmark = zeros(1,num_last_frames_plot);
landmark_History = [];
fig = figure(100);
fig.WindowState = 'maximized';

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 3
        image = im2uint8(imread([recording_path ...
            sprintf('/image_%06d.png',i)]));
    else
        assert(false);
    end
    
    I = image;
    curr_idx = i - bootstrap_frames(2) + 2;
    [S(curr_idx, :), landmark_History] = processFrame(I, I_prev, S(curr_idx - 1, :), landmark_History, continuousParam);
    
   
    I_prev = image;
    
    path = extractPath(S(1:curr_idx,:));
    matched_keypoints = S{curr_idx,1};
    landmark = S{curr_idx,2};
    
    % SUMMARIZING PLOT
    num_tracked_landmark = plotVO(i, path, landmark, landmark_History, size(matched_keypoints,2), num_tracked_landmark, num_last_frames_plot);
    
    
%     if mod(curr_idx,100) == 0
%         if curr_idx < 150
%             S(1:curr_idx,:) = BundleAdjustment(S(1:curr_idx,:), K);
%         else
%             S(curr_idx-150:curr_idx,:) = BundleAdjustment(S(curr_idx-150:curr_idx,:), K);
%         end
%     end

    if BA_flag
        current_keyframe_pos = -S{curr_idx, 6}(:,1:3)'*S{curr_idx, 6}(:,4);
        keyframe_distance = norm(current_keyframe_pos-last_keyframe_pos);
        average_depth_landmarks = mean(S{curr_idx, 2},2);
        average_depth_landmarks = abs(average_depth_landmarks(3)-last_keyframe_pos(3));

        if keyframe_distance/average_depth_landmarks > 0.2
            S(before_last_keyframe_idx:curr_idx,:) = BundleAdjustment(S(before_last_keyframe_idx:curr_idx,:), K);
            last_keyframe_pos = current_keyframe_pos;
            before_last_keyframe_idx = last_keyframe_idx;
            last_keyframe_idx = curr_idx;
        end
    end   
        
    if mod(i, 40) == 0
        disp('ee')
    end
    % Makes sure that plots refresh.    
    pause(0.01);
end

