
clear all; clc; 
addpath(genpath('/home/ynistico/Desktop/experiment_tests/dependencies/')) 
name = 'crawl_new';
my_bag =ros.Bag.load(strcat(name, '.bag'));

%% Extract data from bag file

[msgs, ~] = my_bag.readAll('/state_estimator/Slip_Detection_New');
msgsCell = [ msgs{:} ]; % convert cell array to struct array

% baseline approach
diff_norm_median_LF        = [msgsCell.diff_norm_median_LF       ];
diff_norm_median_RF        = [msgsCell.diff_norm_median_RF       ];
diff_norm_median_RH        = [msgsCell.diff_norm_median_RH       ];
diff_norm_median_LH        = [msgsCell.diff_norm_median_LH       ];
diff_norm_base_imu_kine_LF = [msgsCell.diff_norm_base_imu_kine_LF];
diff_norm_base_imu_kine_RF = [msgsCell.diff_norm_base_imu_kine_RF];
diff_norm_base_imu_kine_LH = [msgsCell.diff_norm_base_imu_kine_LH];
diff_norm_base_imu_kine_RH = [msgsCell.diff_norm_base_imu_kine_RH];
old_flag_1 = [msgsCell.one_leg_slipping];
old_flag_more = [msgsCell.legs_slipping];
mediann = [msgsCell.Median];
norm_stance_vel_base_LF = [msgsCell.norm_stance_vel_base_LF];
norm_stance_vel_base_RF = [msgsCell.norm_stance_vel_base_RF];
norm_stance_vel_base_LH = [msgsCell.norm_stance_vel_base_LH];
norm_stance_vel_base_RH = [msgsCell.norm_stance_vel_base_RH];
th_old_one                 = [msgsCell.th_old_one                ];
th_old_more                = [msgsCell.th_old_more               ];
norm_base_velocity_kine_LF = [msgsCell.norm_base_velocity_kine_LF];
norm_base_velocity_kine_RF = [msgsCell.norm_base_velocity_kine_RF];
norm_base_velocity_kine_RH = [msgsCell.norm_base_velocity_kine_RH];
norm_base_velocity_kine_LH = [msgsCell.norm_base_velocity_kine_LH];
norm_base_velocity_imu_LF  = [msgsCell.norm_base_velocity_imu_LF ];
norm_base_velocity_imu_RF  = [msgsCell.norm_base_velocity_imu_RF ];
norm_base_velocity_imu_RH  = [msgsCell.norm_base_velocity_imu_RH ];
norm_base_velocity_imu_LH  = [msgsCell.norm_base_velocity_imu_LH ];


% new approach 1
norm_desired_foot_vel_base_LF = [msgsCell.norm_desired_foot_vel_base_LF];
norm_desired_foot_vel_base_RF = [msgsCell.norm_desired_foot_vel_base_RF];
norm_desired_foot_vel_base_LH = [msgsCell.norm_desired_foot_vel_base_LH];
norm_desired_foot_vel_base_RH = [msgsCell.norm_desired_foot_vel_base_RH];

norm_actual_foot_vel_base_LF = [msgsCell.norm_actual_foot_vel_base_LF];
norm_actual_foot_vel_base_RF = [msgsCell.norm_actual_foot_vel_base_RF];
norm_actual_foot_vel_base_LH = [msgsCell.norm_actual_foot_vel_base_LH];
norm_actual_foot_vel_base_RH = [msgsCell.norm_actual_foot_vel_base_RH];

diff_norm_des_actual_vel_base_LF = [msgsCell.diff_norm_des_actual_vel_base_LF];
diff_norm_des_actual_vel_base_RF = [msgsCell.diff_norm_des_actual_vel_base_RF];
diff_norm_des_actual_vel_base_LH = [msgsCell.diff_norm_des_actual_vel_base_LH];
diff_norm_des_actual_vel_base_RH = [msgsCell.diff_norm_des_actual_vel_base_RH];

th_new_LF                  = [msgsCell.th_new_LF                 ];
th_new_RF                  = [msgsCell.th_new_RF                 ];
th_new_RH                  = [msgsCell.th_new_RH                 ];
th_new_LH                  = [msgsCell.th_new_LH                 ];

new_flag = [msgsCell.legs_slipping_new];
%

% general 

stance_lf                  = [msgsCell.stance_lf                 ];
stance_rf                  = [msgsCell.stance_rf                 ];
stance_rh                  = [msgsCell.stance_rh                 ];
stance_lh                  = [msgsCell.stance_lh                 ];

vel_foot_lf = [msgsCell.vel_foot_lf];
vel_foot_rf = [msgsCell.vel_foot_rf];
vel_foot_rh = [msgsCell.vel_foot_rh];
vel_foot_lh = [msgsCell.vel_foot_lh];
des_vel_lf  = [msgsCell.des_vel_lf ];
des_vel_rf  = [msgsCell.des_vel_rf ];
des_vel_rh  = [msgsCell.des_vel_rh ];
des_vel_lh  = [msgsCell.des_vel_lh ];
pos_foot_lf = [msgsCell.pos_foot_lf];
pos_foot_rf = [msgsCell.pos_foot_rf];
pos_foot_rh = [msgsCell.pos_foot_rh];
pos_foot_lh = [msgsCell.pos_foot_lh];


%% Extract position of the feet in the horizontal frame

[msgs, ~] = my_bag.readAll('/hyq/debug');
msgsCell = [ msgs{:} ]; % convert cell array to struct array

feet_poss = ["P_Local_HF_LFx"; "P_Local_HF_LFy"; "P_Local_HF_LFz"; "P_Local_HF_RFx"; "P_Local_HF_RFy"; "P_Local_HF_RFz"; "P_Local_HF_LHx"; "P_Local_HF_LHy"; "P_Local_HF_LHz"; "P_Local_HF_RHx"; "P_Local_HF_RHy"; "P_Local_HF_RHz"]; 
base_pos_WF = ["robotPosWFx";"robotPosWFy"];
base_pos_HF = ["actualRobotPosHFx"; "actualRobotPosHFy"; "actualRobotPosHFz"];
desvariables = ["real_time";"time"; "linearSpeedX";feet_poss;base_pos_WF; base_pos_HF];
dataMatrix=[msgsCell(:).data]; % get the values
varnames = [msgsCell(1).name];
%time_debug = [metaCell(:).time];
for name_idx = 1:size(varnames)
    for idx = 1:size(desvariables)
        if (strcmp(varnames(name_idx),desvariables(idx)))
            vardata= dataMatrix(name_idx,:)';
            eval(strcat(varnames{name_idx},'= vardata;'));
            %idx
        end
    end
end




clear msgs msgsCell my_bag name_idx vardata varnames dataMatrix desvariables idx base_pos_HF base_pos_WF feet_poss
save(strcat(name, '.mat'))
disp("Save Mat Sucessfull!")


%%
%plotoptions
ms = load("crawl_slippy_debug.mat");
mns = load("crawl_nonslippy_debug.mat");
seconds_ns = mns.time(1:end)-mns.time(1);
seconds_s = ms.time(1:end)-ms.time(1);

figure(1)
clf
subplot(121)
sgtitle('Actual LF Pose')
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_LFx+mns.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_LFy+mns.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on
%plot(P_Local_HF_RFx+actualRobotPosHFx,P_Local_HF_RFy+actualRobotPosHFy); hold on;

figure(2)
clf
subplot(121)
sgtitle('Actual RF Pose')
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_RFx+mns.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_RFy+mns.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

figure(3)
clf
subplot(121)
sgtitle('Actual RH Pose')
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_RHx+mns.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_RHy+mns.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

figure(4)
clf
subplot(121)
sgtitle('Actual LH Pose')
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_LHx+mns.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(mns.time(1:end)-mns.time(1), mns.P_Local_HF_LHy+mns.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

%%
figure(5)
clf
subplot(121)
sgtitle('Actual LF Pose')
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_LFx+ms.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_LFy+ms.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on
%plot(P_Local_HF_RFx+actualRobotPosHFx,P_Local_HF_RFy+actualRobotPosHFy); hold on;

figure(6)
clf
subplot(121)
sgtitle('Actual RF Pose')
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_RFx+ms.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_RFy+ms.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

figure(7)
clf
subplot(121)
sgtitle('Actual RH Pose')
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_RHx+ms.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_RHy+ms.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

figure(8)
clf
subplot(121)
sgtitle('Actual LH Pose')
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_LHx+ms.actualRobotPosHFx) ; hold on;
%ylim([-0.01 0.01])
xlabel("Time [sec]")
ylabel("Actual Pose along x-axis [m]")
grid on

subplot(122)
plot(ms.time(1:end)-ms.time(1), ms.P_Local_HF_LHy+ms.actualRobotPosHFy); hold on;
xlabel("time [sec]")
ylabel("Actual Pose along y-axis [m]")
%ylim([-0.01 0.01])
grid on

