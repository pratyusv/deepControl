%% user set params
close all;
clc;
clear;

bag_name_dnn = './currBags/rect_dnn_1_20.bag';
bag_name_mpc = './currBags/rect_mpc_1_20.bag';

control_topic_dnn = '/f450/command/roll_pitch_yawrate_thrust';
control_topic = '/f450/command/roll_pitch_yawrate_thrust';

odom_topic = '/f450/ground_truth/odometry';
command_topic = '/f450/command/current_reference';

sys_id_start_time_s = 20;
sys_id_end_time_s = 60;

%% read bag file
% addpath('/home/pratyush/Desktop/Research/packages/catkin_ws/resultPlots/mav_system_identification/read_bags');
path(path, '../read_bags');
path(path, '../helper_functions');


bag_dnn = ros.Bag(bag_name_dnn);
bag_dnn.info

bag_mpc = ros.Bag(bag_name_mpc);
bag_mpc.info



% imu_data = readImu(bag, imu_topic);
attitude_cmd_dnn = readCommandRollPitchYawRateThrust(bag_dnn, control_topic_dnn );
attitude_cmd_mpc = readCommandRollPitchYawRateThrust(bag_mpc, control_topic);

odom_dnn = readOdometry(bag_dnn, odom_topic );
odom_mpc = readOdometry(bag_mpc, odom_topic);

command_dnn= readCommandReference(bag_dnn, command_topic);
command_mpc = readCommandReference(bag_mpc, command_topic);
%%

% imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
% attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);
attitude_cmd_dnn.rpy = vertcat(attitude_cmd_dnn.roll, attitude_cmd_dnn.pitch, attitude_cmd_dnn.yaw_rate);

% t_start = imu_data.t(1);
% imu_data.t = imu_data.t - t_start;
%attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
%attitude_cmd_dnn.t = attitude_cmd_dnn.t - attitude_cmd_dnn.t(1);

odom_dnn.t = odom_dnn.t - odom_dnn.t(1);
odom_mpc.t = odom_mpc.t - odom_mpc.t(1);

command_dnn.t = command_dnn.t - command_dnn.t(1);
command_mpc.t = command_mpc.t - command_mpc.t(1);

[dnn_row, dnn_col] = size(odom_dnn.p);
[mpc_row, mpc_col] = size(odom_mpc.p);
[command_mpc_row, command_mpc_col] = size(command_mpc.p);
[command_dnn_row, command_dnn_col] = size(command_dnn.p);

dnn_size = min(dnn_col, command_dnn_col);
mpc_size = min(mpc_col, command_mpc_col);
pos_size = min(mpc_size, dnn_size);

mpc_start = 1;
mpc_stop  = min(mpc_size, dnn_size);

dnn_start = 1;
dnn_stop  = min(mpc_size,dnn_size);


% dnn_start = 670;
% dnn_stop = 1670;

[dnn_row, dnn_col] = size(odom_dnn.p);
[mpc_row, mpc_col] = size(odom_mpc.p);

%dnn_start = 4100;
%dnn_stop = 7100;

figure(1);
set(gcf,'color','w');
ax = axes;
plot(smooth(odom_dnn.p(1,dnn_start:dnn_stop)), smooth(odom_dnn.p(2,dnn_start:dnn_stop)) *1,'r',  'linewidth', 1);
hold on;
plot(smooth(command_dnn.p(1,dnn_start:dnn_stop)), smooth(command_dnn.p(2,dnn_start:dnn_stop)) *1,'--',  'linewidth', 1);
hold on;
plot(smooth(odom_mpc.p(1,dnn_start:dnn_stop)), smooth(odom_mpc.p(2,dnn_start:dnn_stop)) *1,'b',  'linewidth', 1);


xlabel('x [centimeters]');
ylabel('y [centimeters]');
%title('Y');
legend('DNN','Reference', 'MPC');
grid on;
ax.FontSize = 16;

