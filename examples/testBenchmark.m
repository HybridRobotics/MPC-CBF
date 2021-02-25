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
gamma_list = linspace(0.1, 0.5, 5);
controller_mpc_cbf_list = {};
for i = 1:size(gamma_list, 2)
    new_params = params;
    new_params.N = 5;
    new_params.gamma = gamma_list(i);
    controller_mpc_cbf = MPCCBF(x0, system, new_params);
    controller_mpc_cbf.obs = obs;
    controller_mpc_cbf.sim(time_total);
    controller_mpc_cbf_list{i} = controller_mpc_cbf;
end

%% Computational time benchmark
for i = 1:size(gamma_list, 2)
    controller_mpc_cbf = controller_mpc_cbf_list{i};
    fprintf('Computational time for MPC-CBF3: mean %.3f, std %.3f, min %.3f, max %.3f, input cost %.3f, min dist %f\n',...
    [mean(controller_mpc_cbf.solvertime),...
    std(controller_mpc_cbf.solvertime),...
    min(controller_mpc_cbf.solvertime),...
    max(controller_mpc_cbf.solvertime),...
    controller_mpc_cbf.u_cost,...
    min(controller_mpc_cbf.distlog)]);
end