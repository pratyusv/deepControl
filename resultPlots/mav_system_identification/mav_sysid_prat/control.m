%% user set params
close all;
clc;
clear;

bag_name_dnn = 'inf_dnn1.bag';
bag_name_mpc = 'inf_mpc1.bag';
%imu_topic = '/f_450/mavros/imu/data';
control_topic_dnn = '/f_450/mavros/setpoint_raw/roll_pitch_yawrate_thrust';
% imu_topic = '/f450/imu';
control_topic = '/f_450/command/roll_pitch_yawrate_thrust';
odom_topic = '/f_450/odometry';
command_topic = '/f_450/command/current_reference';

sys_id_start_time_s = 20;
sys_id_end_time_s = 60;

%% read bag file
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

mpc_start = 480;
mpc_stop  = 3480;

dnn_start = 600;
dnn_stop  = 3600;

%% DNN Positio Error
dnn_x_error = 0.0;
for i = dnn_start:5:dnn_stop
    dnn_x_error = dnn_x_error + power((odom_dnn.p(2,i)- command_dnn.p(2,i)),2);
end


dnn_y_error = 0.0;
for i = dnn_start:5:dnn_stop
    dnn_y_error = dnn_y_error + power((odom_dnn.p(1,i)- command_dnn.p(1,i)),2);
end

dnn_error = dnn_x_error + dnn_y_error; 
%--------------------------------------
% MPC Position Error
mpc_x_error = 0.0;
for i = mpc_start:5:mpc_stop
    mpc_x_error = mpc_x_error + power((odom_mpc.p(2,i)- command_mpc.p(2,i)),2);
end


mpc_y_error = 0.0;
for i = mpc_start:5:mpc_stop
    mpc_y_error = mpc_y_error + power((odom_mpc.p(1,i)- command_mpc.p(1,i)),2);
end

mpc_error = mpc_x_error + mpc_y_error;

%% Control Error


dnn_roll_error = 0.0;
for i = dnn_start:5:dnn_stop
    dnn_roll_error = dnn_roll_error + power(attitude_cmd_dnn.roll(i),2);
end

dnn_pitch_error = 0.0;
for i = dnn_start:5:dnn_stop
    dnn_pitch_error = dnn_pitch_error + power(attitude_cmd_dnn.pitch(i),2);
end
%-----------------------------------------------------

mpc_roll_error = 0.0;
for i = mpc_start:5:mpc_stop
    mpc_roll_error = mpc_roll_error + power(attitude_cmd_mpc.roll(i),2);
end


mpc_pitch_error = 0.0;
for i = mpc_start:5:mpc_stop
    mpc_pitch_error = mpc_pitch_error + power(attitude_cmd_mpc.pitch(i),2);
end

mpc_ctrl_cost = mpc_roll_error + mpc_pitch_error;
dnn_ctrl_cost = dnn_roll_error + dnn_pitch_error;

%%




fprintf('dnn_error: %f\n', dnn_error);
% fprintf('dnn_x_error: %f\n\n', dnn_x_error);
% fprintf('dnn_y_error: %f\n\n', dnn_y_error);
% 
fprintf('mpc_error: %f\n', mpc_error);
% fprintf('mpc_x_error: %f\n\n', mpc_x_error);
% fprintf('mpc_y_error: %f\n\n', mpc_y_error);

% fprintf('dnn_roll_error: %f\n', dnn_roll_error);
% fprintf('mpc_roll_error: %f\n', mpc_roll_error);
% 
% fprintf('dnn_pitch_error: %f\n', dnn_pitch_error);
% fprintf('mpc_pitch_error: %f\n', mpc_pitch_error);

fprintf('dnn_ctr_error: %f\n', dnn_ctrl_cost);
fprintf('mpc_ctrl_errors: %f\n', mpc_ctrl_cost);

%dnn_x_error = sum(odom_dnn.p(1,600:3600) - command_dnn.p(1, 600:3600));
%dnn_y_error = sum(odom_dnn.p(2,600:3600) - command_dnn.p(2, 600:3600));

%mpc_x_error = sum(odom_mpc.p(1,280:3280) - command_mpc.p(1, 280:3280));
%mpc_y_error = sum(odom_mpc.p(2,280:3280) - command_mpc.p(2, 280:3280));

%dnn_error = dnn_x_error + dnn_y_error
%mpc_error = mpc_x_error + mpc_y_error







%% plot
figure(1);
ax = axes;
plot(odom_dnn.t(dnn_start:dnn_stop), (attitude_cmd_dnn.roll(dnn_start:dnn_stop)),  'linewidth', 1);
hold on
plot(odom_dnn.t(dnn_start:dnn_stop), (attitude_cmd_mpc.roll(mpc_start:mpc_stop)), 'g','linewidth', 1);

xlabel('time [seconds]');
ylabel(' \phi[radians]');
%title('Y');
legend('DNN', 'MPC');
grid on;
ax.FontSize = 16;




%% 
