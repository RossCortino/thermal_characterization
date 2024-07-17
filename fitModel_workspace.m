clc;
clear;
close all;


weight_W = 0.8; % percent winding RMSE weighted in opimizaiton function
% x0 = [1 1 1 1 1];
x0 = [20 20 0.1 150 10];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0 0 0 0 0];
ub = [];
nonlcon = [];

skip_outputside = load("skip_outputside_5A_compiled_20240313.mat", "data");
skip_motorside = load("skip_motorside_5A_compiled_20240313.mat", "data");

t = {skip_motorside.data.t, skip_outputside.data.t};
q = {skip_motorside.data.q, skip_outputside.data.q};
active = {skip_motorside.data.active, []};
inactive = {skip_motorside.data.inactive, []};
frame = {[], skip_outputside.data.frame};
T_ambient = {skip_motorside.data.ambient, skip_outputside.data.ambient};

skip_model = thermalModel;

% options = optimoptions("fmincon", "Display", "iter");
% skip_model.fit_fmincon(x0, A, b, Aeq, beq, lb, ub, nonlcon, options,...
%     t, q, active, inactive, frame, T_ambient, weight_W, 1);

options = optimoptions('ga','PlotFcn', @gaplotbestf);
skip_model.fit_ga(length(x0), A, b, Aeq, beq, lb, ub, nonlcon, options,...
    t, q, active, inactive, frame, T_ambient, weight_W, 1);

%%

% figure(1);
% subplot(1,2,1);
% plot(MN1005_8A.data.t, (MN1005_8A.data.active + 2*MN1005_8A.data.inactive)/3, 'Color', '#0072BD');
% hold on;
% plot(skip_model.t_train, skip_model.TW_train, 'Color', '#D95319');
% plot(MN1005_8A.data.t, MN1005_8A.data.frame, 'Color', '#0072BD', 'HandleVisibility', 'off');
% plot(skip_model.t_train, skip_model.TH_train, 'Color', '#D95319', 'HandleVisibility', 'off');
% hold off;
% xlabel('Time (sec)');
% ylabel('Temperature (^oC)');
% title('MN1005, Train 8A');
% legend('Data', 'Model');
% 
% subplot(1,2,2);
% plot(MN1005_9A.data.t, (MN1005_9A.data.active + 2*MN1005_9A.data.inactive)/3, 'Color', '#0072BD');
% hold on;
% plot(skip_model.t_test, skip_model.TW_test, 'Color', '#D95319');
% plot(MN1005_9A.data.t, MN1005_9A.data.frame, 'Color', '#0072BD', 'HandleVisibility', 'off');
% plot(skip_model.t_test, skip_model.TH_test, 'Color', '#D95319', 'HandleVisibility', 'off');
% hold off;
% xlabel('Time (sec)');
% title('MN1005, Test 9A');

