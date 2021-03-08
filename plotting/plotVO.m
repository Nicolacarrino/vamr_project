function num_tracked_landmark = plotVO(num_frame, path, landmark, landmark_History, size_matched_keypoints, num_tracked_landmark, num_last_frames_plot)


figure(100)    
subplot(2,2,1)
title(['Current Image ' num2str(num_frame)])

% last frames and landmarks
subplot(1,2,2)
dim_path = size(path,2);

if dim_path <= num_last_frames_plot
    scatter(landmark_History(1,end-min(5000,size(landmark_History,2))+1:end), landmark_History(3,end-min(5000,size(landmark_History,2))+1:end), 10, 'square', 'filled', ...
            'MarkerFaceColor', [0.58 0.58 0.58], 'MarkerEdgeColor', [0.58 0.58 0.58], ...
            'MarkerFaceAlpha', 0.1, 'MarkerEdgeAlpha', 0.1)
    hold on
    scatter(landmark(1,:), landmark(3,:), 10, 'k', 'square', 'filled');
    plot(path(1,1:end),path(3,1:end),'-xb')
    legend('Previously Tracked Landmarks', 'Tracked Landmarks', 'Trajectory')
    hold off
else        
    scatter(landmark_History(1,end-min(5000,size(landmark_History,2))+1:end), landmark_History(3,end-min(5000,size(landmark_History,2))+1:end), 10, 'square', 'filled', ...
            'MarkerFaceColor', [0.58 0.58 0.58], 'MarkerEdgeColor', [0.58 0.58 0.58], ...
            'MarkerFaceAlpha', 0.1, 'MarkerEdgeAlpha', 0.1)
    hold on
    scatter(landmark(1,:), landmark(3,:), 10, 'k', 'square', 'filled');
    plot(path(1,end-num_last_frames_plot:end),path(3,end-num_last_frames_plot:end),'-xb')
    legend('Previously Tracked Landmarks', 'Tracked Landmarks', 'Trajectory')
    hold off
end

xlabel('X')
ylabel('Z')
axis equal
title(['Trajectory of last ' num2str(num_last_frames_plot) ' frames and landmark'])

% tracked landmarks
subplot(2,4,5)
tmp_x = num_frame - num_last_frames_plot + 1 : num_frame;
num_tracked_landmark = [num_tracked_landmark(2:end) size_matched_keypoints];
plot(tmp_x, num_tracked_landmark,'-xb')
xlim([tmp_x(1) tmp_x(end)])
title(['# tracked landmarks over last ' num2str(num_last_frames_plot) ' frames'])

% Full trajectory
subplot(2,4,6)
plot(path(1,:),path(3,:),'-xb')
xlabel('X')
ylabel('Z')
axis equal
title('Full trajectory')




end