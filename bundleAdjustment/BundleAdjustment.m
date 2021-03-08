function states = BundleAdjustment(states, K)

% convertion states in format compatible with BA
poses = [];
num_frames = size(states,1);

landmarks = unique(cell2mat(states(:,2)')','rows','stable');

num_landmarks = size(landmarks,1);
observations = [num_frames; num_landmarks];

for i = 1:num_frames
    single_pose = states{i,6};
    q = rotm2quat(single_pose(:,1:3))';
    t = single_pose(:,4);
    poses = [poses; q; t];
    
    k = size(states{i,2},2);
    [~, ~, ic] = unique([landmarks; states{i,2}'],'rows','stable');
    landmark_indices = ic(num_landmarks+1:end);
    O = [k; states{i,1}(:); landmark_indices];
    
    observations = double([observations; O]);

end

hidden_state = double([poses; reshape(landmarks',[],1)]);

% end convertion states


with_pattern = true;

if with_pattern
    num_frames = observations(1);
    num_observations = (numel(observations)-2-num_frames)/3;
    % Factor 2, one error for each x and y direction.
    num_error_terms = 2 * num_observations;
    % Each error term will depend on one pose (7 entries) and one landmark
    % position (3 entries), so 10 nonzero entries per error term:
    pattern = spalloc(num_error_terms, numel(hidden_state), num_error_terms * 10);
    
    % Fill pattern for each frame individually:
    observation_i = 3;  % iterator into serialized observations
    error_i = 1;  % iterating frames, need another iterator for the error
    for frame_i = 1:num_frames
        num_keypoints_in_frame = observations(observation_i);
        % All errors of a frame are affected by its pose.
        pattern(error_i:error_i+2*num_keypoints_in_frame-1, (frame_i-1)*7+1:frame_i*7) = 1;
        
        % Each error is then also affected by the corresponding landmark.
        landmark_indices = observations(...
            observation_i+2*num_keypoints_in_frame+1:...
            observation_i+3*num_keypoints_in_frame);
        for kp_i = 1:numel(landmark_indices)
            pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
                1+num_frames*7+(landmark_indices(kp_i)-1)*3:...
                num_frames*7+landmark_indices(kp_i)*3) = 1;
        end
        
        observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
        error_i = error_i + 2 * num_keypoints_in_frame;
    end
%     figure(4);
%     spy(pattern);
end

% Also here, using an external error function for clean code.
error_terms = @(hidden_state) ErrorBA(hidden_state, observations, K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', 20);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end



lb = -inf*ones(size(hidden_state,1),1);
ub = inf*ones(size(hidden_state,1),1);
lb(5:7) = hidden_state(5:7);
ub(5:7) = lb(5:7);
hidden_state = lsqnonlin(error_terms, hidden_state, lb, ub, options);



% recompose states

T_C_W = reshape(hidden_state(1:num_frames*7), 7, []);
p_W_landmarks = reshape(hidden_state(num_frames*7+1:end), 3, []);

% Iterator into the observations that are encoded as explained in the 
% problem statement.
observation_i = 2;

for i = 1:num_frames
    single_R_C_W = quat2rotm(T_C_W(1:4, i)'); 
    single_t_C_W = T_C_W(5:7, i);                   
    
    states{i,6} = [single_R_C_W single_t_C_W];
    
    num_frame_observations = observations(observation_i + 1);
    
    landmark_indices = observations(observation_i+2+num_frame_observations * 2:observation_i+1+num_frame_observations * 3);
    
    % Landmarks observed in this specific frame.
    states{i,2} = single(p_W_landmarks(:, landmark_indices));
    
    
    observation_i = observation_i + num_frame_observations * 3 + 1;

end
% end recompose states

end