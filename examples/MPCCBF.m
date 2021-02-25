classdef MPCCBF < handle
    % MPC with distance constraints
    properties
        system
        params
        x0
        x_curr
        time_curr = 0.0
        xlog = []
        ulog = []
        distlog = []
        solvertime = []
        xopenloop = {}
        uopenloop = {}
        u_cost = 0
        obs
    end
    methods
        function self = MPCCBF(x0, system, params)
            % Define MPC_CBF controller
            self.x0 = x0;
            self.x_curr = x0;
            self.system = system;
            self.params = params;
        end
        
        function sim(self, time)
            % Simulate the system until a given time
            xk = self.x_curr;
            while self.time_curr <= time
                % Solve CFTOC
                [~, uk] = self.solveMPCCBF(self.x_curr);
                xk = self.system.A * xk + self.system.B * uk;
                % update system
                self.x_curr = xk;
                self.time_curr = self.time_curr + self.system.dt;
                self.xlog = [self.xlog, xk];
                self.ulog = [self.ulog, uk];
                self.u_cost = self.u_cost + uk'*uk*self.system.dt;
            end
        end
        
        function [xopt, uopt] = solveMPCCBF(self, xk)
            % Solve MPC-CBF
            [feas, x, u, J] = self.solve_cftoc(xk);
            if ~feas
                xopt = [];
                uopt = [];
                return
            else
                xopt = x(:,2);
                uopt = u(:,1);
            end
        end
        
        function [feas, xopt, uopt, Jopt] = solve_cftoc(self, xk)
            % Solve CFTOC
            % extract variables
            N = self.params.N;
            % define variables and cost
            x = sdpvar(4, N+1);
            u = sdpvar(2, N);
            constraints = [];
            cost = 0;
            % initial constraint
            constraints = [constraints; x(:,1) == xk];
            % add constraints and costs
            for i = 1:N
                constraints = [constraints;
                    self.system.xl <= x(:,i) <= self.system.xu;
                    self.system.ul <= u(:,i) <= self.system.uu
                    x(:,i+1) == self.system.A * x(:,i) + self.system.B * u(:,i)];
                cost = cost + x(:,i)'*self.params.Q*x(:,i) + u(:,i)'*self.params.R*u(:,i);
            end
            % add CBF constraints
            for i = 1:N
                pos = self.obs.pos;
                r = self.obs.r ;
                b = (x([1:2],i)-pos)'*((x([1:2],i)-pos)) - r^2;
                b_next = (x([1:2],i+1)-pos)'*((x([1:2],i+1)-pos)) - r^2;
                constraints = [constraints; b_next - b + self.params.gamma * b >= 0];
            end
            % add terminal cost
            cost = cost + x(:,N+1)'*self.params.P*x(:,N+1);
            ops = sdpsettings('solver','ipopt','verbose',0);
            % solve optimization
            diagnostics = optimize(constraints, cost, ops);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt = [];
                uopt = [];
                Jopt = [];
            end
            self.distlog = [self.distlog, sqrt((xk(1:2)-pos)'*(xk(1:2)-pos)-r^2)];
            self.xopenloop{size(self.xopenloop,2)+1} = xopt;
            self.uopenloop{size(self.uopenloop,2)+1} = uopt;
            self.solvertime = [self.solvertime, diagnostics.solvertime];
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function plot(self, figure_name)
            % Plot simulation
            figure('Renderer', 'painters', 'Position', [0 0 400 400]);
            set(gca,'LooseInset',get(gca,'TightInset'));
            hold on;
            % plot closed-loop trajectory
            plot(self.xlog(1,:), self.xlog(2,:), 'ko-',...
                'LineWidth', 1.0, 'MarkerSize', 4);
            % plot open-loop trajectory
            for i = 1:size(self.xopenloop, 2)
                plot(self.xopenloop{i}(1,:), self.xopenloop{i}(2,:),...
                    'k*-.', 'LineWidth', 0.5,'MarkerSize',0.5)
            end
            % plot obstacle
            pos = self.obs.pos;
            r = self.obs.r;
            th = linspace(0,2*pi*100);
            x = cos(th) ; y = sin(th) ;
            plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],...
                'LineWidth', 2);
            plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980],...
                'MarkerSize', 5, 'LineWidth', 2);
            % plot target position
            plot(self.x0(1), self.x0(2), 'db', 'LineWidth', 1);
            plot(0.0, 0.0, 'dr', 'LineWidth', 1);
            h=get(gca,'Children');
            h_legend = legend(h([end]), {'MPC-CBF'}, 'Location', 'SouthEast');
            set(h_legend, 'Interpreter','latex');
            set(gca,'LineWidth', 0.2, 'FontSize', 15);
            grid on
            xlabel('$x (m)$','interpreter','latex','FontSize',20);
            ylabel('$y (m)$','interpreter','latex','FontSize',20);
            xticks(-5:0);
            yticks(-5:0);
            xlim([-5,0.2]);
            ylim([-5,0.2]);
            print(gcf,strcat('figures/',figure_name), '-depsc');
        end
    end
end