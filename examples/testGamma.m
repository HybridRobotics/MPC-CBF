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
params_mpc_cbf.Q = Q;
params_mpc_cbf.R = R;
params_mpc_cbf.P = P;
params_mpc_cbf.N = N;
params_mpc_cbf.gamma = 0.5;

%% Obstacle
obs.pos = [-2; -2.25];
obs.r = 1.5;

%% Simulate MPC-CBF
gamma_list = [0.1, 0.2, 0.3, 1.0]; 
for ind = 1:size(gamma_list, 2)
    fprintf('Run MPC-CBF with gamma %f\n', gamma_list(ind));
    params_mpc_cbf.gamma = gamma_list(ind);
    controller_mpc_cbf_list{ind} = MPCCBF(x0, system, params_mpc_cbf);
    controller_mpc_cbf_list{ind}.obs = obs;
    controller_mpc_cbf_list{ind}.sim(time_total);
end

%% Simulate MPC-DC
params_mpc_dc = params_mpc_cbf;
controller_mpc_dc = MPCDC(x0, system, params_mpc_cbf);
controller_mpc_dc.obs = obs;
controller_mpc_dc.sim(time_total);

%% Visualization benchmark
figure('Renderer', 'painters', 'Position', [0 0 400 400]);
set(gca,'LooseInset',get(gca,'TightInset'));
hold on
for ind = 1:size(gamma_list, 2)
    plot(controller_mpc_cbf_list{ind}.xlog(1,:), controller_mpc_cbf_list{ind}.xlog(2,:),...
        '-', 'LineWidth', 1.0, 'MarkerSize', 4);
end
plot(controller_mpc_dc.xlog(1,:), controller_mpc_dc.xlog(2,:),...
    'k--', 'LineWidth', 1.0, 'MarkerSize', 4);
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
h_legend = legend(h([end, end-1, end-2, end-3, end-4]),...
    {'MPC-CBF ($\gamma=0.1$)', 'MPC-CBF ($\gamma=0.2$)',...
    'MPC-CBF ($\gamma=0.3$)', 'MPC-CBF ($\gamma=1.0$)', 'MPC-DC'}, 'Location', 'SouthEast');
set(h_legend, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 15);
grid on
xlabel('$x (m)$','interpreter','latex','FontSize',20);
ylabel('$y (m)$','interpreter','latex','FontSize',20);
xticks(-5:1);
yticks(-5:1);
xlim([-5,0.2]);
ylim([-5,0.2]);
print(gcf,'figures/benchmark-gamma.eps', '-depsc');
print(gcf,'figures/benchmark-gamma.png', '-dpng', '-r800');