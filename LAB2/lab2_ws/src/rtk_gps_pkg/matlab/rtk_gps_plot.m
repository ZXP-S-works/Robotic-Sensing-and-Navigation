% ten_minutes_bad.bag
% ten_minutes_good.bag
% walking_bad.bag
% walking_good.bag

bag = rosbag('../bagfile/walking_good.bag');
bagselect = select(bag, 'Topic', '/utm_fix');
xyz = timeseries(bagselect,...
    'Pose.Pose.Position.X',...
    'Pose.Pose.Position.Y',...
    'Pose.Pose.Position.Z');
xyz.Data = xyz.Data - min(xyz.Data, [], 1);
figure()
plot(xyz.Data(:, 1)', xyz.Data(:, 2)');
xlabel('x (m)');
ylabel('y (m)');
legend('RTK GPS signal');
axis equal;
figure()
plot3(xyz.Data(:, 1)', xyz.Data(:, 2)', xyz.Data(:, 3)');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend('RTK GPS signal');