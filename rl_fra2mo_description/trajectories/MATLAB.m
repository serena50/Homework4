close,clear,clc

% First plot: CMD_VEL
bag = ros2bagreader('trajectories.bag_0.db3');
topic = '/cmd_vel';
msgs = readMessages(select(bag, 'Topic', topic));

% Extract timestamps and velocities
timestamps_cmd = (1:length(msgs))';
timestamps_cmd = timestamps_cmd * (300/max(timestamps_cmd)); % Scale to 300 seconds

vx = cellfun(@(msg) msg.linear.x, msgs);
vy = cellfun(@(msg) msg.linear.y, msgs);
vz = cellfun(@(msg) msg.linear.z, msgs);

% Create velocity plot
figure;
plot(timestamps_cmd, vx, 'r', timestamps_cmd, vy, 'b', timestamps_cmd, vz, 'g', 'LineWidth', 0.5);
legend('cmd vel');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
title('CMD VEL Components over Time');
grid on;
ylim([0 0.5]);
set(gcf, 'Position', [100, 100, 800, 400]);

% Now plot pose data
topic = '/pose';
msgs_pose = readMessages(select(bag, 'Topic', topic));

% Extract timestamps for pose
timestamps_pose = (1:length(msgs_pose))';
timestamps_pose = timestamps_pose * (300/max(timestamps_pose));

% Extract position
x = cellfun(@(msg) msg.pose.pose.position.x, msgs_pose);
y = cellfun(@(msg) msg.pose.pose.position.y, msgs_pose);
z = cellfun(@(msg) msg.pose.pose.position.z, msgs_pose);

% Create pose plot
figure;
plot(timestamps_pose, x, 'r', timestamps_pose, y, 'b', timestamps_pose, z, 'g', 'LineWidth', 0.5);
legend('X', 'Y', 'Z');
xlabel('Time (sec)');
ylabel('Position (m)');
title('Robot Position over Time');
grid on;
set(gcf, 'Position', [100, 100, 800, 400]);

% Extract quaternions
qw = cellfun(@(msg) msg.pose.pose.orientation.w, msgs_pose);
qx = cellfun(@(msg) msg.pose.pose.orientation.x, msgs_pose);
qy = cellfun(@(msg) msg.pose.pose.orientation.y, msgs_pose);
qz = cellfun(@(msg) msg.pose.pose.orientation.z, msgs_pose);

% Create quaternion plot
figure;
plot(timestamps_pose, qw, 'k', timestamps_pose, qx, 'r', ...
     timestamps_pose, qy, 'g', timestamps_pose, qz, 'b', 'LineWidth', 0.5);
legend('w', 'x', 'y', 'z');
xlabel('Time (sec)');
ylabel('Quaternion Components');
title('Robot Orientation (Quaternions)');
grid on;
set(gcf, 'Position', [100, 100, 800, 400]);

% Additional XY trajectory plot
figure;
plot(x, y, 'b-', 'LineWidth', 2);
hold on;
plot(x(1), y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end), y(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Add waypoints
waypoints = [
    6.5, -1.4;    % Goal 3
    -1.6, -2.5;   % Goal 4
    6.0, 4.0;     % Goal 2
    0.0, 3.0      % Goal 1
];

labels = {'Goal 3', 'Goal 4', 'Goal 2', 'Goal 1'};
plot(waypoints(:,1), waypoints(:,2), 'k*', 'MarkerSize', 10)

% Add labels
for i = 1:length(labels)
    text(waypoints(i,1)+0.2, waypoints(i,2)+0.2, labels{i}, 'FontSize', 12)
end

grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('Robot Trajectory');
legend('Trajectory', 'Start', 'End', 'Waypoints');
axis equal;
set(gcf, 'Position', [100, 100, 800, 800]);