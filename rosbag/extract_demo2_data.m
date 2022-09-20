%% EXTRACT DATA 
clear all; close all; clc;

%% INPUTS
%% solution with fixed stiffness
filename = 'demo_box_no_mod_k.bag';       % Rosbag to extract
% filename = 'demo_fixed_impedance.bag';       % Rosbag to extract
save_name = "fixed_impedance.mat";        % Name of dest mat file

%% Extract data from rosbag

disp('EXTRACTING DATA.');

bag = rosbag(filename);

disp('Opened bag file.');

bag.AvailableTopics

bSel = select(bag,'Topic','/motion_control_dq/info_debug'); 
bSel_traj = select(bag,'Topic','/motion_control_dq/dq_trajectory'); 
bSel_comp = select(bag,'Topic','/motion_control_dq/compliant_traj'); 

msgStructs = readMessages(bSel,'DataFormat','struct'); %%read message from dual arm control node
msgStructs_traj = readMessages(bSel_traj,'DataFormat','struct'); %%read message from dual arm  trajectory gen node
msgStructs_comp = readMessages(bSel_comp,'DataFormat','struct'); %%read message from dual

len_msg = length(msgStructs); 
len_msg_traj = length(msgStructs_traj); 
len_msg_comp = length(msgStructs_comp); 

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

%% Variables from admittance loop
stamp_comp = zeros(1,len_msg_comp);
pos_comp_rel = zeros(3,len_msg_comp); %compliant relative position
frel = zeros(3,len_msg_comp); %relative force 
stiffness = zeros(3,len_msg_comp);  %relative stiffnes 
t_bias_2 = double(msgStructs_comp{1}.Header.Stamp.Sec)+...
double(msgStructs_comp{1}.Header.Stamp.Nsec)*10^-9;

for i = 1 : length(msgStructs_comp)
    ros_t = msgStructs_comp{i}.Header.Stamp;
    stamp_comp(i) = double(ros_t.Sec)+double(ros_t.Nsec)*10^-9;
    stamp_comp(i) = stamp_comp(i)-t_bias_2;
    pos_comp_rel(:,i) = msgStructs_comp{i}.PosRelC(1:3);
    frel(:,i) = msgStructs_comp{i}.FrF(1:3);
    stiffness(:,i) = msgStructs_comp{i}.Stiff(1:3);
  
end

disp('Read Compliant variables.');


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
max_t = max([stamp_traj(end), stamp_comp(end)]); 
dt = max([mean(diff(stamp_traj)),mean(diff(stamp_comp))]); 
time  =  0 : dt : max_t;


pos_nom_pp  =  zeros(size(pos_nom,1),size(time,2));
pos_comp_rel_pp  =  zeros(size(pos_comp_rel,1),size(time,2));
f_rel_pp = zeros(size(frel,1),size(time,2)); 
stiff_pp = zeros(size(stiffness,1),size(time,2)); 

for i  =  1 : size(time,2)
    [~, ix]  =  min(abs(stamp_traj-time(i)));
    pos_nom_pp(:,i)  =  pos_nom(:, ix);
    
    [~, ix]  =  min(abs(stamp_comp-time(i)));
    pos_comp_rel_pp(:,i)  =  pos_comp_rel(:, ix);
    
    [~, ix]  =  min(abs(stamp_comp-time(i)));
    f_rel_pp(:,i)  =  frel(:, ix);
    
    [~, ix]  =  min(abs(stamp_comp-time(i)));
    stiff_pp(:,i) = stiffness(:,ix); 
end

disp('Scaled time.');

%% Save the extracted data

save_path = fullfile(pwd,save_name);

if isfile(save_path)
    msg = 'WARNING: The specified file already exists.';
    disp(msg);
end
    save(save_path,'fl','fr','stiff_pp','f_rel_pp','pos_comp_rel_pp','time','pos_nom_pp','pos_r','pos_1','pos_2');

%% Plotting demo with trivial solution
% load("fixed_impedance.mat");
%%Define color
red = [0.8 0.2 0.2]; 
gr = [0.3 0.6 0.3];
or = [0.9 0.5 0.1]; 

%% forces
tt = 0:0.001:17.3; 
sizet = size(tt,2);
lim2 = 17.3; 


for i = 13900:sizet
   fl(2,i) = fl(2,i)*0.25;
   fr(2,i) = fr(2,i)*0.35;
end

f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,fl(2,1:sizet)*-1,'LineWidth',1.5,'Color',red);
xlim([0,lim2])
ylabel('$f_1/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2, 1, 2)
grid on
hold on
plot(tt,fr(2,1:sizet)*-1,'LineWidth',1.5);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([0,lim2])
ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% arms position y-axis
f = figure;
f.Renderer = 'painters';
subplot(2, 1, 1)
grid on
hold on
plot(tt,pos_1(2,1:sizet),'LineWidth',1.5,'Color',or);
grid on; hold on
xlim([0,lim2])
%%plot(tt,0.92*ones(sizet,1),'r--','LineWidth',1.5); 
ylabel('$y_1/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2, 1, 2)
grid on
hold on
plot(tt,pos_2(2,1:sizet),'LineWidth',1.5,'Color',gr);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
grid on; hold on
xlim([0,lim2])
%%plot(tt,0.63*ones(sizet,1),'r--','LineWidth',1.5); 
ylabel('$y_2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Relative position cfr 
f = figure;
f.Renderer = 'painters';
grid on
hold on
plot(time(1,:),pos_nom_pp(3,:),'LineWidth',1.5);
grid on
hold on
plot(time(1,:),pos_comp_rel_pp(3,:),'LineWidth',1.5);
hold on 
grid on
plot(tt,0.26*ones(sizet,1),'LineStyle','--','LineWidth',1.5); 
hold on 
grid on
plot(tt,pos_r(3,1:sizet),'LineWidth',1.5,'Color',[0.5 0 0.5]);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([0 lim2])
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('ref','comp','contact point','curr','Interpreter', 'latex', 'FontSize', 10)

%% Relative translational stiffness
% f = figure;
% f.Renderer = 'painters';
% grid on
% hold on
% plot(time(1,:),stiff_pp(3,:),'LineWidth',1.5,'Color',[0.5 0 0.5]);
% xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([0 20])
% ylabel('$K/\mathrm{Nm}$', 'Interpreter', 'latex', 'FontSize', 12)

%% relative force z-axis
f = figure;
f.Renderer = 'painters';
grid on
hold on
plot(time(1,:),f_rel_pp(3,:),'LineWidth',1.5);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([0 lim2])
ylabel('$f_r/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

%% Analysis
f = figure;
f.Renderer = 'painters';
subplot(2,2,1);
grid on
hold on
plot(tt,fl(2,1:sizet)*-1,'LineWidth',1.5,'Color',red);
xlim([0,lim2])
ylabel('$f/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)


subplot(2,2,2);
grid on
hold on
plot(tt,fr(2,1:sizet)*-1,'LineWidth',1.5);
xlim([0,lim2])
% ylabel('$f_2/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2,2,3);
grid on
hold on
plot(tt,pos_1(2,1:sizet),'LineWidth',1.5,'Color',or);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
xlim([0,lim2])
% xlim([1.8,4])
ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(2,2,4);
grid on
hold on
plot(tt,pos_2(2,1:sizet),'LineWidth',1.5,'Color',gr);
xlim([0,lim2])
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlim([1.8,4])
% ylabel('$y_2/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)


