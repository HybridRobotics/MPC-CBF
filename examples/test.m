close all
clear

%% General Flags
run_dclf_dcbf = true;
display_dclf_dcbf = true;
run_mpc_cbf_one = true;
display_mpc_cbf_one = true;
run_mpc_cbf_multiple = true;
run_mpc_cbf_multiple = true;
run_mpc_dc = true;
display_mpc_dc = true;

%% Setup and Parameters
x0 = [-5; -5; 0; 0];
time_total = 30.0;
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

%% DCLF-DCBF parameters
p = 1e3;
params_dclf_dcbf.P = P;
params_dclf_dcbf.H = R;
params_dclf_dcbf.p = p;
params_dclf_dcbf.alpha = 1.0;
params_dclf_dcbf.gamma = 0.4;

%% MPC-CBF parameters
params_mpc_cbf.Q = Q;
params_mpc_cbf.R = R;
params_mpc_cbf.P = P;
params_mpc_cbf.N = N;
params_mpc_cbf.gamma = 0.4;

%% MPC-DC parameters
params_mpc_dc.Q = Q;
params_mpc_dc.R = R;
params_mpc_dc.P = P;
params_mpc_dc.N = N;

%% Obstacle
obs.pos = [-2; -2.25];
obs.r = 1.5;

%% Simulate DCLF-DCBF
if run_dclf_dcbf
    fprintf('Run DCLF-DCBF\n');
    controller_dclf_dcbf = DCLFDCBF(x0, system, params_dclf_dcbf);
    controller_dclf_dcbf.obs = obs;
    controller_dclf_dcbf.sim(time_total);
end

%% Display DCLF-DCBF simulation
if display_dclf_dcbf
    % Plot simulation
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot trajectory
    plot(controller_dclf_dcbf.xlog(1,:), controller_dclf_dcbf.xlog(2,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
    % plot obstacle
    pos = controller_dclf_dcbf.obs.pos;
    r = controller_dclf_dcbf.obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(controller_dclf_dcbf.x0(1), controller_dclf_dcbf.x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    h=get(gca,'Children');
    h_legend = legend(h([end]), {'DCLF-DCBF'}, 'Location', 'SouthEast');
    set(h_legend, 'Interpreter','latex');
    set(gca,'LineWidth', 0.2, 'FontSize', 15);
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xticks(-5:0);
    yticks(-5:0);
    xlim([-5,0.2]);
    ylim([-5,0.2]);
    print(gcf,'figures/dclf-dcbf-avoidance.eps', '-depsc');
    print(gcf,'figures/dclf-dcbf-avoidance.png', '-dpng', '-r800');
end

%% Simulate MPC-CBF with N=1;
params_mpc_cbf.N = 1;
if run_mpc_cbf_one
    fprintf('Run MPC-CBF\n');
    controller_mpc_cbf_one = MPCCBF(x0, system, params_mpc_cbf);
    controller_mpc_cbf_one.obs = obs;
    controller_mpc_cbf_one.sim(time_total);
end

%% Display MPC-CBF simulation with N=1
if display_mpc_cbf_one
    % Plot simulation
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot trajectory
    plot(controller_mpc_cbf_one.xlog(1,:), controller_mpc_cbf_one.xlog(2,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
    % plot obstacle
    pos = controller_mpc_cbf_one.obs.pos;
    r = controller_mpc_cbf_one.obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(controller_mpc_cbf_one.x0(1), controller_mpc_cbf_one.x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    h=get(gca,'Children');
    h_legend = legend(h([end]), {'MPC-CBF ($N=1$)'}, 'Location', 'SouthEast');
    set(h_legend, 'Interpreter','latex');
    set(gca,'LineWidth', 0.2, 'FontSize', 15);
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xticks(-5:0);
    yticks(-5:0);
    xlim([-5,0.2]);
    ylim([-5,0.2]);
    print(gcf, 'figures/mpc-cbf-avoidance-one-step.eps', '-depsc');
    print(gcf, 'figures/mpc-cbf-avoidance-one-step.png', '-dpng', '-r800');
end

%% Simulate MPC-CBF with other N
params_mpc_cbf.N = 8;
if run_mpc_cbf_one
    fprintf('Run MPC-CBF\n');
    controller_mpc_cbf_multiple = MPCCBF(x0, system, params_mpc_cbf);
    controller_mpc_cbf_multiple.obs = obs;
    controller_mpc_cbf_multiple.sim(time_total);
end

%% Display MPC-CBF simulation with other N
if display_mpc_cbf_one
    % Plot simulation
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot trajectory
    plot(controller_mpc_cbf_multiple.xlog(1,:), controller_mpc_cbf_multiple.xlog(2,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
    % plot obstacle
    pos = controller_mpc_cbf_multiple.obs.pos;
    r = controller_mpc_cbf_multiple.obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(controller_mpc_cbf_multiple.x0(1), controller_mpc_cbf_multiple.x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    h=get(gca,'Children');
    h_legend = legend(h([end]), {'MPC-CBF ($N=8$)'}, 'Location', 'SouthEast');
    set(h_legend, 'Interpreter','latex');
    set(gca,'LineWidth', 0.2, 'FontSize', 15);
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xticks(-5:0);
    yticks(-5:0);
    xlim([-5,0.2]);
    ylim([-5,0.2]);
    print(gcf, 'figures/mpc-cbf-avoidance-several-steps.eps', '-depsc');
    print(gcf, 'figures/mpc-cbf-avoidance-several-steps.png', '-dpng', '-r800');
end

%% Simulate MPC-DC
if run_mpc_dc
    fprintf('Run MPC-DC\n');
    controller_mpc_dc = MPCDC(x0, system, params_mpc_dc);
    controller_mpc_dc.obs = obs;
    controller_mpc_dc.sim(time_total);
end

%% Display MPC-DC simulation
if display_mpc_dc
    % Plot simulation
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot trajectory
    plot(controller_mpc_dc.xlog(1,:), controller_mpc_dc.xlog(2,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
    % plot obstacle
    pos = controller_mpc_dc.obs.pos;
    r = controller_mpc_dc.obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(controller_mpc_dc.x0(1), controller_mpc_dc.x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    h=get(gca,'Children');
    h_legend = legend(h([end]), {'MPC-DC ($N=8$)'}, 'Location', 'SouthEast');
    set(h_legend, 'Interpreter','latex');
    set(gca,'LineWidth', 0.2, 'FontSize', 15);
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xticks(-5:0);
    yticks(-5:0);
    xlim([-5,0.2]);
    ylim([-5,0.2]);
    print(gcf, 'figures/mpc-dc-avoidance.eps', '-depsc');
    print(gcf, 'figures/mpc-dc-avoidance.png', '-dpng', '-r800');
end