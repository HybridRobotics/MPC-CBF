close all
clear

%% Setup and Parameters
x0 = [-5; -5; 0; 0];
time_total = 20.0;
dt = 0.2;
P = 100*eye(4);
Q = 10*eye(4);
R = eye(2);
N = 8;
xmin = [-5; -5; -5; -5];
xmax = [5; 5; 5; 5];
umin = [-1; -1];
umax = [1; 1];

%% Discrete-time double integrator 2D
system.dt = dt;
system.A = [1 0 dt 0;
    0 1 0 dt;
    0 0 1 0;
    0 0 0 1];
system.B = [0.5*dt^2 0;
    0 0.5*dt^2;
    dt 0;
    0 dt];
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

%% MPC-CBF parameters
params.Q = Q;
params.R = R;
params.P = P;
params.N = N;
params.gamma = 0.5;

%% Obstacle
obs.pos = [-2; -2.25];
obs.r = 1.5;

%% Simulate MPC-CBF
params.N = 5;
params.gamma = 0.25;
controller_mpc_cbf_1 = MPCCBF(x0, system, params);
controller_mpc_cbf_1.obs = obs;
controller_mpc_cbf_1.sim(time_total);

%% Simulate MPC-CBF with lower gamma
params.gamma = 0.20;
controller_mpc_cbf_2 = MPCCBF(x0, system, params);
controller_mpc_cbf_2.obs = obs;
controller_mpc_cbf_2.sim(time_total);

params.gamma = 0.15;
controller_mpc_cbf_3 = MPC_CBF(x0, system, params);
controller_mpc_cbf_3.obs = obs;
controller_mpc_cbf_3.sim(time_total);

%% Simulate MPC-DC

% The problem is infeasible at N=5
% params.N = 5;
% controller_mpc_dc_0 = MPCDC(x0, system, params);
% controller_mpc_dc_0.obs = obs;
% controller_mpc_dc_0.sim(time_total);

params.N = 7;
controller_mpc_dc_1 = MPCDC(x0, system, params);
controller_mpc_dc_1.obs = obs;
controller_mpc_dc_1.sim(time_total);

params.N = 15;
controller_mpc_dc_2 = MPCDC(x0, system, params);
controller_mpc_dc_2.obs = obs;
controller_mpc_dc_2.sim(time_total);

%%
params.N = 30;
controller_mpc_dc_3 = MPCDC(x0, system, params);
controller_mpc_dc_3.obs = obs;
controller_mpc_dc_3.sim(time_total);

%% Visualization benchmark
figure('Renderer', 'painters', 'Position', [0 0 400 400]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
plot(controller_mpc_cbf_3.xlog(1,:), controller_mpc_cbf_3.xlog(2,:),...
    '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.0, 'MarkerSize', 4);
plot(controller_mpc_cbf_2.xlog(1,:), controller_mpc_cbf_2.xlog(2,:),...
    '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.0, 'MarkerSize', 4);
plot(controller_mpc_cbf_1.xlog(1,:), controller_mpc_cbf_1.xlog(2,:),...
    '-', 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth', 1.0, 'MarkerSize', 4);
plot(controller_mpc_dc_1.xlog(1,:), controller_mpc_dc_1.xlog(2,:),...
    '--', 'Color', [0.4940, 0.1840, 0.5560], 'LineWidth', 1.0, 'MarkerSize', 4);
plot(controller_mpc_dc_2.xlog(1,:), controller_mpc_dc_2.xlog(2,:),...
    '--', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 1.0, 'MarkerSize', 4);
plot(controller_mpc_dc_3.xlog(1,:), controller_mpc_dc_3.xlog(2,:),...
    '--', 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth', 1.0, 'MarkerSize', 4);
% plot obstacle
pos = obs.pos;
r = obs.r;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],...
    'LineWidth', 2);
plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980],...
    'MarkerSize', 5, 'LineWidth', 2);
% plot target position
plot(x0(1), x0(2), 'db', 'LineWidth', 1);
plot(0.0, 0.0, 'dr', 'LineWidth', 1);
h=get(gca,'Children');
h_legend = legend(h([end, end-1, end-2, end-3, end-4, end-5]),...
    {'MPC-CBF ($N = 5, \gamma = 0.15$)', 'MPC-CBF ($N = 5, \gamma = 0.20$)',...
    'MPC-CBF ($N = 5, \gamma = 0.25$)', 'MPC-DC ($N = 7$)', 'MPC-DC ($N = 15$)',...
    'MPC-DC ($N = 30$)'}, 'Location', 'SouthEast');
set(h_legend, 'Interpreter','latex', 'FontSize', 10);
set(gca,'LineWidth', 0.2, 'FontSize', 15);
grid on
xlabel('$x (m)$','interpreter','latex','FontSize',20);
ylabel('$y (m)$','interpreter','latex','FontSize',20);
xticks(-5:0);
yticks(-5:0);
xlim([-5,0.2]);
ylim([-5,0.2]);
print(gcf,'figures/benchmark-horizon.eps', '-depsc');
print(gcf,'figures/benchmark-horizon.png', '-dpng', '-r800');

%% Computational time benchmark
figure('Renderer', 'painters', 'Position', [0 0 800 400]);
xlabel('Computational time (s)', 'interpreter', 'latex', 'FontSize', 16);
ylabel('Percentage', 'interpreter', 'latex', 'FontSize', 16);
hold on
% [N_mpc_3, edges_mpc_3] = histcounts(controller_mpc_cbf_3.solvertime, 10, 'Normalization', 'probability');
% plot(edges_mpc_3(1:end-1), N_mpc_3, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1);
% [N_mpc_2, edges_mpc_2] = histcounts(controller_mpc_cbf_2.solvertime, 10, 'Normalization', 'probability');
% plot(edges_mpc_2(1:end-1), N_mpc_2, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1);
[N_mpc_1, edges_mpc_1] = histcounts(controller_mpc_cbf_1.solvertime, 10, 'Normalization', 'probability');
plot(edges_mpc_1(1:end-1), N_mpc_1, '-', 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth', 1);
[N_dc_1, edges_dc_1] = histcounts(controller_mpc_dc_1.solvertime, 10, 'Normalization', 'probability');
plot(edges_dc_1(1:end-1), N_dc_1, '--', 'Color', [0.4940, 0.1840, 0.5560], 'LineWidth', 1);
[N_dc_2, edges_dc_2] = histcounts(controller_mpc_dc_2.solvertime, 10, 'Normalization', 'probability');
plot(edges_dc_2(1:end-1), N_dc_2, '--', 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 1);
[N_dc_3, edges_dc_3] = histcounts(controller_mpc_dc_3.solvertime, 10, 'Normalization', 'probability');
plot(edges_dc_3(1:end-1), N_dc_3, '--', 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth', 1);
h=get(gca,'Children');
h_legend = legend(h([end, end-1, end-2, end-3]),...
    {'MPC-CBF ($N = 5, \gamma = 0.25$)',...
    'MPC-DC ($N = 7$)',...
    'MPC-DC ($N = 15$)',...
    'MPC-DC ($N = 30$)'}, 'Location', 'NorthEast');
set(h_legend, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 15);
print(gcf,'figures/benchmark-horizon-computational-time.eps', '-depsc');
print(gcf,'figures/benchmark-horizon-computational-time.png', '-dpng', '-r800');
%% Computational time table
fprintf('Computational time for MPC-CBF3: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_cbf_3.solvertime),...
    std(controller_mpc_cbf_3.solvertime),...
    min(controller_mpc_cbf_3.solvertime),...
    max(controller_mpc_cbf_3.solvertime),...
    controller_mpc_cbf_3.u_cost,...
    min(controller_mpc_cbf_3.distlog)]);

fprintf('Computational time for MPC-CBF2: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_cbf_2.solvertime),...
    std(controller_mpc_cbf_2.solvertime),...
    min(controller_mpc_cbf_2.solvertime),...
    max(controller_mpc_cbf_2.solvertime),...
    controller_mpc_cbf_2.u_cost,...
    min(controller_mpc_cbf_2.distlog)]);

fprintf('Computational time for MPC-CBF1: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_cbf_1.solvertime),...
    std(controller_mpc_cbf_1.solvertime),...
    min(controller_mpc_cbf_1.solvertime),...
    max(controller_mpc_cbf_1.solvertime),...
    controller_mpc_cbf_1.u_cost,...
    min(controller_mpc_cbf_1.distlog)]);

% fprintf('Computational time for MPC-DC0: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
%     [mean(controller_mpc_dc_0.solvertime),...
%     std(controller_mpc_dc_0.solvertime),...
%     min(controller_mpc_dc_0.solvertime),...
%     max(controller_mpc_dc_0.solvertime),...
%     controller_mpc_dc_0.u_cost,...
%     min(controller_mpc_dc_0.distlog)]);

fprintf('Computational time for MPC-DC1: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_dc_1.solvertime),...
    std(controller_mpc_dc_1.solvertime),...
    min(controller_mpc_dc_1.solvertime),...
    max(controller_mpc_dc_1.solvertime),...
    controller_mpc_dc_1.u_cost,...
    min(controller_mpc_dc_1.distlog)]);

fprintf('Computational time for MPC-DC2: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_dc_2.solvertime),...
    std(controller_mpc_dc_2.solvertime),...
    min(controller_mpc_dc_2.solvertime),...
    max(controller_mpc_dc_2.solvertime),...
    controller_mpc_dc_2.u_cost,...
    min(controller_mpc_dc_2.distlog)]);

fprintf('Computational time for MPC-DC3: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_dc_3.solvertime),...
    std(controller_mpc_dc_3.solvertime),...
    min(controller_mpc_dc_3.solvertime),...
    max(controller_mpc_dc_3.solvertime),...
    controller_mpc_dc_3.u_cost,...
    min(controller_mpc_dc_3.distlog)]);