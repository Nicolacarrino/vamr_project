function [S, landmark_History] = processFrame(I, I_prev, S_prev, landmark_History, continuousParam)

S = cell(1,6);

points_prev = S_prev{1}';
landmark_prev = S_prev{2};
points_cand_prev = S_prev{3}';
points_cand_first_observations = S_prev{4}';
points_cand_pose = S_prev{5};
camera_pose_prev = S_prev{6};

pointTracker_associated_kepoints = vision.PointTracker('NumPyramidLevels', continuousParam.NumPyramidLevels, ...
   'MaxBidirectionalError', continuousParam.MaxBidirectionalError, 'BlockSize', continuousParam.BlockSize, ...
   'MaxIterations', continuousParam.MaxIterations);
initialize(pointTracker_associated_kepoints, points_prev, I_prev) 
[points, point_validity] = pointTracker_associated_kepoints(I);

matched_keypoints = points(point_validity,:)';
tmp = landmark_prev;
landmark_prev = landmark_prev(:,point_validity);
tmp(:,point_validity) = [];
landmark_History = [landmark_History tmp];


pointTracker_candidate_kepoints = vision.PointTracker('NumPyramidLevels', continuousParam.NumPyramidLevels, ...
   'MaxBidirectionalError', continuousParam.MaxBidirectionalError, 'BlockSize', continuousParam.BlockSize, ...
   'MaxIterations', continuousParam.MaxIterations);
initialize(pointTracker_candidate_kepoints, points_cand_prev, I_prev) 
[points_cand, point_cand_validity] = pointTracker_candidate_kepoints(I);

p_cand = points_cand(point_cand_validity,:)'; %candidates that were able to be tracked
p_cand_first_observation = points_cand_first_observations(point_cand_validity,:)';
p_cand_pose = points_cand_pose(:,point_cand_validity);


figure(100)
subplot(2,2,1)
hold off
imshow(I);
hold on;
plot(points(point_validity, 1), points(point_validity, 2), 'rx', 'Linewidth', 1);
plot([points_prev(point_validity,1)'; points(point_validity,1)'], ...
     [points_prev(point_validity,2)'; points(point_validity,2)'], 'g-', 'Linewidth', 1);


[Orientation, Location, inlierIdx] = estimateWorldCameraPose(matched_keypoints',landmark_prev',continuousParam.Intrinsics, ...
    'MaxNumTrials', continuousParam.MaxNumTrials, 'Confidence', continuousParam.Confidence, ...
    'MaxReprojectionError', continuousParam.MaxReprojectionError);

[R_C_W, t_C_W] = cameraPoseToExtrinsics(Orientation, Location);

S{1} = matched_keypoints(:,inlierIdx);
S{2} = landmark_prev(:,inlierIdx);
S{6} = [R_C_W t_C_W'];

% refine the pose just computed
if continuousParam.poseRefinementFlag
    S = poseRefinement(S, continuousParam.K);
    R_C_W = S{6}(:,1:3);
    t_C_W = S{6}(:,4)';
end

landmark_camera = R_C_W'*landmark_prev + t_C_W'; 

median_dim = median(landmark_camera, 2);
idx = landmark_camera(3,:) > 0;
idx = idx & landmark_camera(3,:) < continuousParam.RangeDepth*median_dim(3);


landmark_prev = landmark_prev(:,idx);
matched_keypoints = matched_keypoints(:,idx);

landmark_camera = R_C_W'*landmark_prev + t_C_W';


S{1} = matched_keypoints;
S{2} = landmark_prev;

if size(p_cand,2) ~= 0
    bearings_current = continuousParam.K\[p_cand; ones(1,size(p_cand,2))];
    bearings_first_observation = continuousParam.K\[p_cand_first_observation; ones(1,size(p_cand,2))];

    angle = acosd( dot(bearings_current,bearings_first_observation)./(vecnorm(bearings_current).*vecnorm(bearings_first_observation)));

    delete_ind = true(size(p_cand,2),1);

    angle_index = 1:length(angle);
    angle_index = angle_index(abs(angle) > continuousParam.Threshold_angle);

    for i = 1:length(angle_index)
        idx = angle_index(i);
        T_vec = reshape(p_cand_pose(:,idx),3,4);
        R_first_obs = T_vec(1:3,1:3);
        t_first_obs = T_vec(:,4)';
        keypoint_1 = p_cand(:,idx);
        keypoint_2 = p_cand_first_observation(:,idx);

        % triangulate new landmark
        camMatrix1 = cameraMatrix(continuousParam.Intrinsics, R_C_W, t_C_W);
        camMatrix2 = cameraMatrix(continuousParam.Intrinsics, R_first_obs, t_first_obs);
        
        new_landmark = triangulate(keypoint_1', keypoint_2', camMatrix1, camMatrix2);
        
        new_landmark_camera = R_C_W'*new_landmark' + t_C_W'; 
        
        median_dim = median(landmark_camera, 2);
               
        if new_landmark_camera(3) > 0 && new_landmark_camera(3) < continuousParam.RangeDepth*median_dim(3)
            % add new keypoint and landmark to the associated ones
            S{1} = [S{1} p_cand(:,idx)];
            S{2} = [S{2} new_landmark'];
            delete_ind(idx) = false;    
        end
    end
    
fprintf('%d new keypoint added\n', length(delete_ind) - nnz(delete_ind));

% search for new candidate keypoints
new_candidates = detectHarrisFeatures(I, 'MinQuality', continuousParam.MinQuality, 'FilterSize', continuousParam.FilterSize);
keypoints_image = new_candidates.Location;

dist_p = pdist2(matched_keypoints', keypoints_image, 'squaredeuclidean', 'Smallest', 1);
dist_c = pdist2(p_cand', keypoints_image, 'squaredeuclidean', 'Smallest', 1);

new_candidate_keypoints = keypoints_image(dist_p > continuousParam.Threshold_same_point & dist_c > continuousParam.Threshold_same_point,:);


p_cand = p_cand(:, delete_ind);
p_cand_first_observation = p_cand_first_observation(:, delete_ind);
p_cand_pose = p_cand_pose(:, delete_ind);


if size(new_candidate_keypoints,1) ~= 0
    % add new candidates to state
    p_cand = [p_cand new_candidate_keypoints'];
    p_cand_first_observation = [p_cand_first_observation new_candidate_keypoints'];
    p_cand_pose = [p_cand_pose repmat(reshape([R_C_W t_C_W'],[],1),1,size(new_candidate_keypoints,1))];
end

S{3} = p_cand;
S{4} = p_cand_first_observation;
S{5} = p_cand_pose;

end
