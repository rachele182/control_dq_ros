%% PLOTS

clear all; close all; clc;

disp('EXTRACTING DATA.');

%% Extract data from rosbag
filename = '_2021-01-15-12-24-56.bag';

bag = rosbag(filename);

disp('Opened bag file.');

% Impedance
topic_data = select(bag,...
    'Topic', '/variable_impedance_controller/desired_impedance');
msg_struct = topic_data.readMessages;

len_msg = length(msg_struct);
stamp_imp = zeros(1,len_msg);
stiffness = zeros(36,len_msg);
damping = zeros(36,len_msg);
t_bias = double(msg_struct{1}.Header.Stamp.Sec)+...
    double(msg_struct{1}.Header.Stamp.Nsec)*10^-9;
for i = 1 : length(msg_struct)
    ros_t = msg_struct{i}.Header.Stamp;
    stamp_imp(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_imp(i) = stamp_imp(i)-t_bias;
    stiffness(:,i) = msg_struct{i}.StiffnessMatrix(1:36);
    damping(:,i) = msg_struct{i}.DampingMatrix(1:36);
end

disp('Read Impedance.');

% State
topic_data = select(bag,...
    'Topic', '/variable_impedance_controller/robot_state');
msg_struct = topic_data.readMessages;

len_msg = length(msg_struct);
stamp_sta = zeros(1,len_msg);
mass = zeros(49,len_msg);
jacobian = zeros(42,len_msg);
t_bias = double(msg_struct{1}.Header.Stamp.Sec)+...
    double(msg_struct{1}.Header.Stamp.Nsec)*10^-9;
for i = 1 : length(msg_struct)
    ros_t = msg_struct{i}.Header.Stamp;
    stamp_sta(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_sta(i) = stamp_sta(i)-t_bias;
    mass(:,i) = msg_struct{i}.MassMatrix(1:49);
    jacobian(:,i) = msg_struct{i}.JacobianMatrix(1:42);
end

disp('Read State.');

% Forces
topic_data = select(bag,...
    'Topic', '/franka_state_controller/F_ext');
msg_struct = topic_data.readMessages;

len_msg = length(msg_struct);
stamp_for = zeros(1,len_msg);
force = zeros(6,len_msg);
t_bias = double(msg_struct{1}.Header.Stamp.Sec)+...
    double(msg_struct{1}.Header.Stamp.Nsec)*10^-9;
for i = 1 : length(msg_struct)
    ros_t = msg_struct{i}.Header.Stamp;
    stamp_for(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_for(i) = stamp_for(i)-t_bias;
    force(:,i) = [msg_struct{i}.Wrench.Force.X;
        msg_struct{i}.Wrench.Force.Y;
        msg_struct{i}.Wrench.Force.Z;
        msg_struct{i}.Wrench.Torque.X;
        msg_struct{i}.Wrench.Torque.Y;
        msg_struct{i}.Wrench.Torque.Z];
end

disp('Read Forces.');

% Errors
topic_data = select(bag,...
    'Topic', '/variable_impedance_controller/pos_error');
msg_struct = topic_data.readMessages;

len_msg = length(msg_struct);
stamp_err = zeros(1,len_msg);
error = zeros(6,len_msg);
t_bias = double(msg_struct{1}.Header.Stamp.Sec)+...
    double(msg_struct{1}.Header.Stamp.Nsec)*10^-9;
for i = 1 : length(msg_struct)
    ros_t = msg_struct{i}.Header.Stamp;
    stamp_err(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_err(i) = stamp_err(i)-t_bias;
    error(:,i) = [msg_struct{i}.Twist.Linear.X;
        msg_struct{i}.Twist.Linear.Y;
        msg_struct{i}.Twist.Linear.Z;
        msg_struct{i}.Twist.Linear.X;
        msg_struct{i}.Twist.Linear.Y;
        msg_struct{i}.Twist.Linear.Z];
end

disp('Read Errors.');

% Bounds
topic_data = select(bag,...
    'Topic', '/impedance_planning/bounds');
msg_struct = topic_data.readMessages;

len_msg = length(msg_struct);
stamp_bou = zeros(1,len_msg);
bounds = zeros(6,len_msg);
t_bias = double(msg_struct{1}.Header.Stamp.Sec)+...
    double(msg_struct{1}.Header.Stamp.Nsec)*10^-9;
for i = 1 : length(msg_struct)
    ros_t = msg_struct{i}.Header.Stamp;
    stamp_bou(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_bou(i) = stamp_bou(i)-t_bias;
    bounds(:,i) = [msg_struct{i}.Twist.Linear.X;
        msg_struct{i}.Twist.Linear.Y;
        msg_struct{i}.Twist.Linear.Z;
        msg_struct{i}.Twist.Linear.X;
        msg_struct{i}.Twist.Linear.Y;
        msg_struct{i}.Twist.Linear.Z];
end

disp('Read Bounds.');

%% Post processing

% Getting minimum resolution time
max_t = max([stamp_bou(end), stamp_err(end), stamp_for(end),...
    stamp_imp(end), stamp_sta(end)]);
dt = max([mean(diff(stamp_bou)), mean(diff(stamp_err)), ...
    mean(diff(stamp_for)),mean(diff(stamp_imp)), mean(diff(stamp_sta))]);
time  =  0 : dt : max_t;

stiffness_pp  =  zeros(size(stiffness,1),size(time,2));
damping_pp  =  zeros(size(damping,1),size(time,2));
mass_pp  =  zeros(size(mass,1),size(time,2));
jacobian_pp  =  zeros(size(jacobian,1),size(time,2));
force_pp  =  zeros(size(force,1),size(time,2));
error_pp  =  zeros(size(error,1),size(time,2));
bounds_pp  =  zeros(size(bounds,1),size(time,2));

for i  =  1 : size(time,2)
    [~, ix]  =  min(abs(stamp_imp-time(i)));
    stiffness_pp(:,i)  =  stiffness(:, ix);
    damping_pp(:,i)  =  damping(:, ix);
    
    [~, ix]  =  min(abs(stamp_sta-time(i)));
    mass_pp(:,i)  =  mass(:, ix);
    jacobian_pp(:,i)  =  jacobian(:, ix);
    
    [~, ix]  =  min(abs(stamp_for-time(i)));
    force_pp(:,i)  =  force(:, ix);
    
    [~, ix]  =  min(abs(stamp_err-time(i)));
    error_pp(:,i)  =  error(:, ix);
    
    [~, ix]  =  min(abs(stamp_bou-time(i)));
    bounds_pp(:,i)  =  bounds(:, ix);
end

disp('Scaled time.');

% Getting vectors from data
tout = time;
e_out = error_pp;
var_bounds = bounds_pp;

% Getting matrices and computing cost and norms
lambda_var = zeros(1,size(time,2));
d_var = zeros(1,size(time,2));
k_var = zeros(1,size(time,2));
cost = zeros(1,size(time,2));
f_ext = zeros(1,size(time,2));
for i  =  1 : size(time,2)
    J = reshape(jacobian_pp(:,i),6,7);
    M = reshape(mass_pp(:,i),7,7);
    lambda_var(:,i) = norm(inv(J*inv(M)*J.'),'fro');
    D = reshape(damping_pp(:,i),6,6);
    K = reshape(stiffness_pp(:,i),6,6);
    cost(:,i) = norm(D,'fro') + norm(K,'fro');
    d_var(:,i) = norm(D,'fro');
    k_var(:,i) = norm(K,'fro');
    f_ext(:,i) = norm(force_pp(:,i));
end

disp('Extracted required quantities.');

% Resample cost to the same rate (needed is cost is sent to workspace
% alone - TODO: solve this )
% cost_pp = resample(cost,length(tout),length(cost));
% Remove lambda from cost
cost_pp = cost(1, :);

%% Plotting

disp('PLOTTING.');

% forces
figure
hold on
plot(tout, f_ext(1,:),'Color','#A2142F','linewidth',2);
legend('$\mathrm F_{\mathrm {ext}_x}$','Interpreter','latex', 'FontSize', 15)
% title('External Force', 'FontSize', 18, 'Interpreter','latex')
xlabel('time [s]');
ylabel('force [N]');
set(gca,'Color',[240 240 240]/255);
grid on
drawnow

% errors
figure
hold on
axis ([0 max_t -0.15 0.15])
patch([0 20 20 0], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
patch([20 37 37 20], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [180 180 180]/255, 'FaceAlpha', 0.3)
patch([37 max_t max_t 37], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
plot(tout, e_out(1,:),tout, e_out(2,:),'g','linewidth',2);
plot(tout, var_bounds(1,:),'k--','linewidth',2);
plot(tout, -var_bounds(1,:),'k--','linewidth',2);
% yline(bound(1),'k--','linewidth',2);
% yline(-bound(1),'k--','linewidth',2); 
% legend('Strict req','Relax req','$\tilde x_x$','$\tilde x_y$',...
%     '$b_i$','Interpreter','latex', 'FontSize', 15)
legend('Strict req','Relaxed req','Strict req','$\tilde x_x$',...
    '$\tilde x_y$','$b_i$','Interpreter','latex','NumColumns',3)
% title('Tracking Error', 'FontSize', 18, 'Interpreter','latex')
xlabel('time [s]');
ylabel('tracking error [m]');
% set(gca,'Color',[240 240 240]/255);
grid on
drawnow

% impedance
figure
hold on
axis ([0 max_t 0 max(k_var)+0.05*max(k_var)])
patch([0 20 20 0], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
patch([20 37 37 20], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [180 180 180]/255, 'FaceAlpha', 0.3)
patch([37 max_t max_t 37], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
plot(tout, lambda_var(1,:),'Color','#D95319','linewidth',2);
plot(tout, d_var(1,:),'Color','#77AC30','linewidth',2);
plot(tout, k_var(1,:),'Color','#7E2F8E','linewidth',2);
legend('Strict req','Relaxed req','Strict req','$|| \mathbf \Lambda ||$','$|| \mathbf D ||$',...
    '$|| \mathbf K ||$','Interpreter','latex','NumColumns',3)
% title('Planned Impedance', 'FontSize', 18, 'Interpreter','latex')
xlabel('time [s]');
ylabel('inertia - stiffness - damping');
grid on
drawnow

% cost
figure
hold on
axis ([0 max_t min(cost)-0.05*min(cost) max(cost)+0.05*max(cost)])
patch([0 20 20 0], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
patch([20 37 37 20], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [180 180 180]/255, 'FaceAlpha', 0.3)
patch([37 max_t max_t 37], [max(ylim) max(ylim) min(ylim) min(ylim)],...
    [150 200 150]/255, 'FaceAlpha', 0.3)
plot(tout, cost_pp(1,:),'Color','#0072BD','linewidth',2);
legend('Strict req','Relaxed req','Strict req','$J$','Interpreter','latex',...
    'NumColumns',3);
% title('Cost', 'FontSize', 18, 'Interpreter','latex')
xlabel('time [s]');
ylabel('cost');
grid on
drawnow

disp('Finished.');
