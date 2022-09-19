%% EXTRACT DATA 
clear all; close all; clc;

%% INPUTS
%% trivial solution with thight grasp
filename = 'demo_box_no_adm.bag';       % Rosbag to extract
save_name = "no_admittance.mat";        % Name of dest mat file

%% Extract data from rosbag

disp('EXTRACTING DATA.');

bag = rosbag(filename);

disp('Opened bag file.');

bag.AvailableTopics; 

bSel = select(bag,'Topic','/motion_control_dq/info_debug'); 
bSel_traj = select(bag,'Topic','/motion_control_dq/dq_trajectory'); 

msgStructs = readMessages(bSel,'DataFormat','struct'); %%read message from dual arm control node
msgStructs_traj = readMessages(bSel_traj,'DataFormat','struct'); %%read message from dual arm  trajectory gen node

len_msg = length(msgStructs); 
len_msg_traj = length(msgStructs_traj); 

%% Variables from trajectory gen node
stamp_traj= zeros(1,len_msg_traj);
pos_nom = zeros(3,len_msg_traj); %nominal relative position
t_bias = double(msgStructs_traj{1}.Header.Stamp.Sec)+...
double(msgStructs_traj{1}.Header.Stamp.Nsec)*10^-9;

%% relative position reference 
for i = 1 : length(msgStructs_traj)
    ros_t = msgStructs_traj{i}.Header.Stamp;
    stamp_traj(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_traj(i) = stamp_traj(i)-t_bias;
    pos_nom(:,i) = msgStructs_traj{i}.PositionR(1:3);
   
end

disp('Read Relative position.');

%% Variables from control node
fl = zeros(3,len_msg); %left arm
fr = zeros(3,len_msg); %right arm
pos_r = zeros(3,len_msg); %rel position
pos_1 = zeros(3,len_msg); %arm1 position
pos_2 = zeros(3,len_msg); %arm2 position

%% INTERNAL FORCES, RELATIVE POS, EE
for i = 1 : length(msgStructs)
    fl(:,i) = msgStructs{i}.FExtHatL(1:3);
    fr(:,i) = msgStructs{i}.FExtHatR(1:3);
    pos_r(:,i) = msgStructs{i}.RelPos(1:3);
    pos_1(:,i) = msgStructs{i}.Pos1(1:3);
    pos_2(:,i) = msgStructs{i}.Pos2(1:3);
    
end

disp('Read Forces.');

%% Post processing

% Getting minimum resolution time
max_t = stamp_traj(end);
dt = mean(diff(stamp_traj));
time  =  0 : dt : max_t;

%% Save the extracted data

save_path = fullfile(pwd,save_name);

if isfile(save_path)
    msg = 'WARNING: The specified file already exists.';
    disp(msg);
%else
    save(save_path,'fl','fr','time','pos_nom','pos_r','pos_1','pos_2');
end

%% Plotting demo with trivial solution
load("no_admittance.mat");

%%Define color
red = [0.8 0.2 0.2]; 
gr = [0.3 0.6 0.3];
or = [0.9 0.5 0.1]; 


%% forces
tt = 0:0.001:15; 
sizet = size(tt,2); 
f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,fl(2,1:sizet)*-1,'LineWidth',1.5,'Color',red);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([1.8,4])
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2, 1, 2)
grid on
hold on
plot(tt,fr(2,1:sizet)*-1,'LineWidth',1.5);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([1.8,4])
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% arms position y-axis
tt = 0:0.001:15; 
sizet = size(tt,2); 
f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,pos_1(2,1:sizet),'LineWidth',1.5,'Color',or);
grid on; hold on
%%plot(tt,0.92*ones(sizet,1),'r--','LineWidth',1.5); 
% xlim([1.8,4])
ylabel('$y_1/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2, 1, 2)
grid on
hold on
plot(tt,pos_2(2,1:sizet),'LineWidth',1.5,'Color',gr);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([1.8,4])
grid on; hold on
%%plot(tt,0.63*ones(sizet,1),'r--','LineWidth',1.5); 
ylabel('$y_2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Relative position cfr 
f = figure;
f.Renderer = 'painters';
grid on
hold on
plot(time(1,:),pos_nom(3,:),'LineWidth',1.5);
grid on
hold on
plot(tt,pos_r(3,1:sizet),'LineWidth',1.5,'Color',[0.5 0 0.5]);
hold on 
grid on
plot(tt,0.26*ones(sizet,1),'LineStyle','--','LineWidth',1); 
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([0 15])
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('ref','curr','contact point','Interpreter', 'latex', 'FontSize', 10)


%% Analysis
f = figure;
f.Renderer = 'painters';
subplot(2,2,1);
grid on
hold on
plot(tt,fl(2,1:sizet)*-1,'LineWidth',1.5,'Color',red);
ylabel('$f/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('left arm','Interpreter', 'latex', 'FontSize', 10)

subplot(2,2,2);
grid on
hold on
plot(tt,fr(2,1:sizet)*-1,'LineWidth',1.5);
% ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('right arm','Interpreter', 'latex', 'FontSize', 10)


subplot(2,2,3);
grid on
hold on
plot(tt,pos_1(2,1:sizet),'LineWidth',1.5,'Color',or);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([1.8,4])
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('left pos','Interpreter', 'latex', 'FontSize', 10)

subplot(2,2,4);
grid on
hold on
plot(tt,pos_2(2,1:sizet),'LineWidth',1.5,'Color',gr);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('right pos','Interpreter', 'latex', 'FontSize', 10)

% xlim([1.8,4])
% ylabel('$y_2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)





