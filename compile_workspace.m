clc;
clear;
close all;

trial = "skip_motorside_5A";
current_step_val = 5;

date = "20240313";

unit = "C";
F_flag = 0;

interp_T = 0.01;

T_active_csv = trial + "_" + date + "_" + unit + "__active.csv";
T_inactive_csv = trial + "_" + date + "_" + unit + "__inactive.csv";
T_frame_csv = [];
V_mat = trial + "_" + date + ".mat";

% T_active_csv = [];
% T_inactive_csv = [];
% T_frame_csv = trial + "_" + date + "_" + unit + "__ground.csv";
% V_mat = trial + "_" + date + ".mat";

data = compileData_thermalTest(current_step_val, T_active_csv, T_inactive_csv, T_frame_csv, F_flag, V_mat, interp_T);

%%

figure(1);
yyaxis left;
plot(data.t, data.active);
hold on;
plot(data.t, data.inactive);
% plot(data.t, data.frame);
hold off;
ylabel('Temperature (^oC)');
yyaxis right;
plot(data.t, data.q);
ylabel('Heat (W)');
xlabel('Time (sec)');

%%
save(trial + "_compiled_" + date, 'data');