%% user set params
%% user set params

clc;
clear all;
close all;

bag_name_dnn = 'currBags/rect_dnn_1_95.bag';
bag_name_mpc = 'currBags/rect_mpc_1_95.bag';
%bag_name_ref = 'step_mpc.bag';

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

% bag_ref = ros.Bag(bag_name_ref);
% bag_ref.info

%%  STEP INPUT
% dnn_start = 500;
% dnn_stop = 1500;
% 
% mpc_start = 133;
% mpc_stop = 1133;

%% INFINITY
dnn_start = 1365;
dnn_stop = 1865 ;

mpc_start = 1500;
mpc_stop = 3000;


%%
% attitude_cmd_dnn = readCommandRollPitchYawRateThrust(bag_dnn, control_topic_dnn );
% attitude_cmd_mpc = readCommandRollPitchYawRateThrust(bag_mpc, control_topic);

command = readCommandReference(bag_dnn, command_topic);

odom_dnn = readOdometry(bag_dnn, odom_topic );
odom_mpc = readOdometry(bag_mpc, odom_topic);

odom_dnn.t = odom_dnn.t - odom_dnn.t(1);
odom_mpc.t = odom_mpc.t - odom_mpc.t(1);
command.t = command.t - command.t(1);

[dnn_row, dnn_col] = size(odom_dnn.p);
dnn_x_error = 0.0;

for i = 1:5:dnn_col
    dnn_x_error = dnn_x_error + power((odom_dnn.p(2,i)- command.p(2,i)),2);
end

print(dnn_x_error);





figure(1);
ax = axes;

%plot(smooth(odom_dnn.p(1,dnn_start:dnn_stop))*1, smooth(odom_dnn.p(2,dnn_start:dnn_stop))*1,  'linewidth', 1);
%hold on
%plot(smooth(odom_mpc.p(1,mpc_start:mpc_stop))*1, smooth(odom_mpc.p(2,mpc_start:mpc_stop))*1, 'g','linewidth', 1);


plot(smooth(odom_dnn.p(1,:))*1, smooth(odom_dnn.p(2,:))*1,  'linewidth', 1);
hold on
plot(smooth(odom_mpc.p(1,:))*1, smooth(odom_mpc.p(2,:))*1, 'g','linewidth', 1);
hold on
plot(smooth(command.p(1,:))*1, smooth(command.p(2,:))*1, '--','linewidth', 1);



% xlabel('time [seconds]');
% ylabel('y [centimeter]');
% 
% legend('DNN', 'MPC');
grid on;
% ax.FontSize = 16;

% figure(1);
% x_mpc= [];
% y_mpc= [];
% 
% x_dnn= [];
% y_dnn= [];
% 
% y_ref  = [];
% %Create axes
% set(gcf, 'Position', get(0,'Screensize'));
% axes1 = axes('Position',[0.13 0.11 0.250355276907001 0.815]);
% hold(axes1,'on');
% 
% plot(smooth(command.t(1:2201)), smooth(command.p(2,1450:3650)),'--', 'linewidth', 1,'Color',[222/255 ,125/255,0]);
% % hold on
% 
% 
% 
% for i = 1:1000
%     x_mpc = [odom_mpc.p(1,mpc_start +i), x_mpc];
%     x_dnn = [odom_dnn.p(1,dnn_start + i), x_dnn];
%     
%     y_mpc = [odom_mpc.p(2,mpc_start +i), y_mpc];
%     y_dnn = [odom_dnn.p(2,dnn_start + i), y_dnn];
%     
%     
%     
%     pause(0.0001);
%     plot(x_dnn, y_dnn,'b');
%     hold on
%     plot(x_mpc/25.4,y_mpc,'g');
% %     hold on
% %     plot(x_dnn, y_ref, '--', 'linewidth', 1,'Color',[222/255 ,125/255,0]);
%        
% %     xlim([0,43]);
% %     ylim([-1.1,0.1]);
%     xlabel('time[seconds]');
%     ylabel('y[meters]');
%     grid on;
%     %legend('Ref','DNN', 'MPC');
%     %ax.FontSize = 16;
% end



    