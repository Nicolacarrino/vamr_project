function error_terms = ErrorPoseRefinement(hidden_state, observations, landmarks, K)
% hidden_state is given by a [q' t' q' t'] so no landmark refinement
% where q is a quaternion, t is a translation
% observation = [n m O O O]


num_frames = observations(1);
T_C_W = reshape(hidden_state(1:num_frames*7), 7, []);
p_W_landmarks = landmarks;

error_terms = [];
% Iterator into the observations that are encoded as explained in the 
% problem statement.
observation_i = 2;

for i = 1:num_frames
    single_R_C_W = quat2rotm(T_C_W(1:4, i)'); 
    single_t_C_W = T_C_W(5:7, i);                   
                
    num_frame_observations = observations(observation_i + 1);
    
    keypoints = reshape(observations(observation_i+2:observation_i+1+num_frame_observations * 2), 2, []);
    
    landmark_indices = observations(observation_i+2+num_frame_observations * 2:observation_i+1+num_frame_observations * 3);
    
    % Landmarks observed in this specific frame.
    p_W_L = p_W_landmarks(:, landmark_indices);
    
    % Transforming the observed landmarks into the camera frame for projection.
    p_C_L = single_R_C_W' * p_W_L + single_t_C_W;
    
    % From exercise 1.
    projections = projectPoints(p_C_L, K);
    
    error_terms = double([error_terms keypoints-projections]);
    
    observation_i = observation_i + num_frame_observations * 3 + 1;

end

end

