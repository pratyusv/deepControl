%% user set params

%close all;
% clc;
% clear;
bag_name_mpc = 'hover_battery_mpc.bag';
control_topic_mpc = '/f_450/mavros/setpoint_raw/roll_pitch_yawrate_thrust';
battery_topic = '/f_450/mavros/battery';


%% read bag file
path
path(path, '../read_bags');
path(path, '../helper_functions');


bag_mpc = ros.Bag(bag_name_mpc);
bag_mpc.info


%%
battery_mpc=  readBatteryState(bag_mpc, battery_topic );
thrust_mpc = readCommandRollPitchYawRateThrust(bag_mpc, control_topic_mpc);

battery_mpc.t = battery_mpc.t - battery_mpc.t(1);
thrust_mpc.t = thrust_mpc.t - thrust_mpc.t(1);


%% plot
figure(1);
ax = axes;
plot(battery_mpc.t, (battery_mpc.v),'r', 'linewidth', 1);
hold on
plot(thrust_mpc.t, (thrust_mpc.thrust(3,:)),'b',  'linewidth', 1);
xlabel('time [seconds]');
ylabel('voltage');
legend('Voltage','Thrust');

%figure(2);
%plot(battery_mpc.t, smooth(battery_mpc.p), '--','linewidth', 2);
%xlabel('time [seconds]');
%ylabel('percentage ');

%title('Y');
grid on;
% ax.FontSize = 16;

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
