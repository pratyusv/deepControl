%% user set params

%  bag_name = '/run/user/1000/gvfs/sftp:host=172.27.20.73,user=pratyush/home/pratyush/Desktop/offline_mpc/f450/iros_graphs/bag/infinity_run/infinity_run.bag';
% odom_topic = '/f_450/odometry';
% control_topic = '/f_450/mavros/setpoint_raw/roll_pitch_yawrate_thrust';
% ref_topic = '/f_450/command/current_reference';

bag_name = '/run/user/1000/gvfs/sftp:host=172.27.20.73,user=pratyush/home/pratyush/Desktop/offline_mpc/f450/iros_graphs/bag/sim_infinity_run/infinity_run_sim_dnn.bag';
control_topic = '/f450/command/roll_pitch_yawrate_thrust';
ref_topic = '/f450/command/current_reference';
odom_topic = '/f450/ground_truth/odometry';


bag_name_mpc = '/run/user/1000/gvfs/sftp:host=172.27.20.73,user=pratyush/home/pratyush/Desktop/offline_mpc/f450/iros_graphs/bag/sim_infinity_run/infinity_run_sim_mpc.bag';


sys_id_start_time_s = 5;
sys_id_end_time_s = 40;

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

close all;
clc;

bag = ros.Bag(bag_name);
bag.info

bag_mpc = ros.Bag(bag_name_mpc);
bag_mpc.info

% imu_data = readImu(bag, imu_topic);
current_ref = readCommandReference(bag, ref_topic);
odometry    = readOdometry(bag, odom_topic);
attitude_cmd = readCommandRollPitchYawRateThrust(bag, control_topic);
%%
current_ref_mpc = readCommandReference(bag_mpc, ref_topic);
odometry_mpc    = readOdometry(bag_mpc, odom_topic);
attitude_cmd_mpc = readCommandRollPitchYawRateThrust(bag_mpc, control_topic);

% imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
% attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);

% t_start = imu_data.t(1);
% imu_data.t = imu_data.t - t_start;
current_ref.t = current_ref.t - current_ref.t(1);
odometry.t = odometry.t - odometry.t(1);
%% plot
figure(1);
axes1 = axes;
lim_init = 1
lim_end  = 5000
plot( odometry.p(2,lim_init:lim_end), odometry.p(1,lim_init:lim_end),'r','linewidth', 0.1);
hold on;
plot( current_ref.p(2,lim_init:lim_end),current_ref.p(1,lim_init:lim_end),'g--', 'linewidth', 2);
hold on;
plot( odometry_mpc.p(2,lim_init:lim_end), odometry_mpc.p(1,lim_init:lim_end),'b','linewidth', 0.1);
xlabel('Y [meters]','FontSize',24);
ylabel('X [meters]','FontSize',24);
legend('DNN', 'Refrence');
grid on;

% 
% % Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[0 45]);
% % Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-0.4 0.4]);
% % Uncomment the following line to preserve the Z-limits of the axes
% % zlim(axes1,[-1 1]);
% box(axes1,'on');
% grid(axes1,'on');
% % Set the remaining axes properties
% set(axes1,'FontSize',24.2277407631738);
% % Create legend
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.787393164542364 0.699954720902378 0.112980767081563 0.0985054320291332],...
%     'FontSize',21.8049666868564);



% figure(2);
% axes1 = axes;
% plot(imu_data.t, imu_data.rpy(2,:), 'linewidth', 2);
% hold on;
% plot(attitude_cmd.t, attitude_cmd.rpy(2,:), '--', 'linewidth', 2);
% xlabel('time [sec]','FontSize',24);
% ylabel('\theta [rad]','FontSize',24);
% 
% legend('\theta imu', '\theta cmd');
% grid on;
% 
% 
% % Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[0 45]);
% % Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-0.4 0.4]);
% % Uncomment the following line to preserve the Z-limits of the axes
% % zlim(axes1,[-1 1]);
% box(axes1,'on');
% grid(axes1,'on');
% % Set the remaining axes properties
% set(axes1,'FontSize',24.2277407631738);
% % Create legend
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.787393164542364 0.699954720902378 0.112980767081563 0.0985054320291332],...
%     'FontSize',21.8049666868564);
% 
% 
% 
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
% 
% %% sysid
% 
% attitude_cmd.rpy_interp = zeros(size(imu_data.rpy));
% attitude_cmd.rpy_interp(1,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(1,:), imu_data.t);
% attitude_cmd.rpy_interp(2,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(2,:), imu_data.t);
% attitude_cmd.rpy_interp(3,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(3,:), imu_data.t);
% 
% attitude_cmd.t = imu_data.t;
% 
% imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
% imu_data.rpy = imu_data.rpy(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
% 
% attitude_cmd.t = attitude_cmd.t(attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
% attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
% 
% dt = mean(diff(imu_data.t));
% roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
% pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);
% 
% roll_tf = tfest( roll_data, 1, 0);
% pitch_tf = tfest( pitch_data, 1, 0);
% disp('======================');
% disp('roll 1st order dynamics'); 
% % roll_tf
% fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
% fprintf('roll_time_constant: %f\n\n', -1/pole(roll_tf));
% fprintf('fit percentage: %f %%\n', roll_tf.Report.Fit.FitPercent);
% disp('----------------------');
% disp('pitch 1st order dynamics'); 
% % pitch_tf
% fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
% fprintf('pitch_time_constant: %f\n\n', -1/pole(pitch_tf));
% fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);
% 
% %2nd order
% roll_tf = tfest( roll_data, 2, 0);
% pitch_tf = tfest( pitch_data, 2, 0);
% disp('----------------------');
% disp('roll 2st order dynamics:'); 
% % roll_tf
% [Wn, damping] = damp(roll_tf);
% fprintf('roll_gain: %f\n\n', dcgain(roll_tf));
% fprintf('roll_damping_coef: %f\n\n', damping(1));
% fprintf('roll_natural_freq: %f\n\n', Wn(1));
% fprintf('fit percentage: %f\n', roll_tf.Report.Fit.FitPercent);
% 
% disp('----------------------');
% disp('pitch 2st order dynamics:'); 
% % pitch_tf
% [Wn, damping] = damp(pitch_tf);
% fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf));
% fprintf('pitch_damping_coef: %f\n\n', damping(1));
% fprintf('pitch_natural_freq: %f\n\n', Wn(1));
% fprintf('fit percentage: %f\n', pitch_tf.Report.Fit.FitPercent);
% 
% 
% csvwrite("imu_t.csv",imu_data.t)
% %% 
