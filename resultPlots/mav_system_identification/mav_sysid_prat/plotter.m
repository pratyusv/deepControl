%% user set params

bag_name_dnn = './currBags/rect_dnn.bag';
bag_name_mpc = './currBags/rect_mpc.bag';
%imu_topic = '/f_450/mavros/imu/data';
control_topic_dnn = '/f450/command/roll_pitch_yawrate_thrust';
% imu_topic = '/f450/imu';
control_topic = '/f450/command/roll_pitch_yawrate_thrust';
odom_topic = '/f450/ground_truth/odometry';
command_topic = '/f450/command/current_reference';

sys_id_start_time_s = 20;
sys_id_end_time_s = 60;

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

close all;
clc;

bag_dnn = ros.Bag(bag_name_dnn);
bag_dnn.info

bag_mpc = ros.Bag(bag_name_mpc);
bag_mpc.info



% imu_data = readImu(bag, imu_topic);
% attitude_cmd_dnn = readCommandRollPitchYawRateThrust(bag_dnn, control_topic_dnn );
% attitude_cmd_mpc = readCommandRollPitchYawRateThrust(bag_mpc, control_topic);

odom_dnn = readOdometry(bag_dnn, odom_topic );
odom_mpc = readOdometry(bag_mpc, odom_topic);

command_dnn= readCommandReference(bag_dnn, command_topic);
command_mpc = readCommandReference(bag_mpc, command_topic);
%%

% imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
% attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);
% attitude_cmd_dnn.rpy = vertcat(attitude_cmd_dnn.roll, attitude_cmd_dnn.pitch, attitude_cmd_dnn.yaw_rate);

% t_start = imu_data.t(1);
% imu_data.t = imu_data.t - t_start;
%attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
%attitude_cmd_dnn.t = attitude_cmd_dnn.t - attitude_cmd_dnn.t(1);

odom_dnn.t = odom_dnn.t - odom_dnn.t(1);
odom_mpc.t = odom_mpc.t - odom_mpc.t(1);

command_dnn.t = command_dnn.t - command_dnn.t(1);
command_mpc.t = command_mpc.t - command_mpc.t(1);

%% plot
figure(1);
ax = axes;
stop_value = 3000;
plot(odom_dnn.t(600:stop_value), smooth(odom_dnn.p(2,600:stop_value))*1,  'linewidth', 1);
hold on
plot(odom_dnn.t(600:stop_value), (command_dnn.p(2,600:stop_value))*1, '--','linewidth', 2);
hold on
plot(odom_dnn.t(600:stop_value), smooth(odom_mpc.p(2,280:stop_value))*1, 'g','linewidth', 1);

xlabel('time [seconds]');
ylabel('y [centimeter]');
%title('Y');
legend('DNN','Reference', 'MPC');
grid on;
ax.FontSize = 16;

% figure(2);
% ax = axes;
% plot(imu_data.t, imu_data.rpy(2,:), 'linewidth', 2);
% hold on;
% plot(attitude_cmd.t, attitude_cmd.rpy(2,:), '--', 'linewidth', 2);
% xlabel('time');
% ylabel('\theta [rad]');
% title('pitch angle');
% legend('\theta imu', '\theta cmd');
% grid on;
% ax.FontSize = 16;
% 
% figure(3);
% ax = axes;
% plot(imu_data.t, imu_data.a(3, :), 'linewidth', 2);
% hold on;
% plot(attitude_cmd.t, attitude_cmd.thrust, '--', 'linewidth', 2);
% xlabel('time');
% ylabel('???');
% title('Thrust');
% legend('Acceleration', 'Commanded thrust');
% grid on;
% ax.FontSize = 16;



%% 
