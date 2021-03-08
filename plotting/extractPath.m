function path = extractPath(states)


num_frames = size(states,1);
path = zeros(3, num_frames);

for i = 1:num_frames
   pose = states{i,6};
   R_C_W = pose(:,1:3);
   t_C_W = pose(:,4);
   path(:,i) = -R_C_W*t_C_W;

end

end