function state = poseRefinement(state, K)

% convertion states in format compatible with poseRefinement
poses = [];
num_frames = size(state,1);

landmarks = unique(cell2mat(state(:,2)')','rows','stable');

num_landmarks = size(landmarks,1);
observations = [num_frames; num_landmarks];

for i = 1:num_frames
    single_pose = state{i,6};
    q = rotm2quat(single_pose(:,1:3))';
    t = single_pose(:,4);
    poses = [poses; q; t];
    
    k = size(state{i,2},2);
    [~, ~, ic] = unique([landmarks; state{i,2}'],'rows','stable');
    landmark_indices = ic(num_landmarks+1:end);
    O = [k; state{i,1}(:); landmark_indices];
    
    observations = double([observations; O]);

end

hidden_state = poses;
landmarks = double(landmarks');
% end convertion states


with_pattern = true;

if with_pattern
    num_frames = observations(1);
    num_observations = (numel(observations)-2-num_frames)/3;
    % Factor 2, one error for each x and y direction.
    num_error_terms = 2 * num_observations;
    % Each error term will depend on one pose (7 entries), 
    % so 7 nonzero entries per error term:
    pattern = ones(num_error_terms, 7);
    
end


% Also here, using an external error function for clean code.
error_terms = @(hidden_state) ErrorPoseRefinement(hidden_state, observations, landmarks, K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', 20);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end

hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);



% recompose states

T_C_W = reshape(hidden_state(1:num_frames*7), 7, []);

% Iterator into the observations that are encoded as explained in the 
% problem statement.
observation_i = 2;

for i = 1:num_frames
    single_R_C_W = quat2rotm(T_C_W(1:4, i)'); 
    single_t_C_W = T_C_W(5:7, i);                   
    
    state{i,6} = [single_R_C_W single_t_C_W];
    num_frame_observations = observations(observation_i + 1);
    
    observation_i = observation_i + num_frame_observations * 3 + 1;

end
% end recompose states

end