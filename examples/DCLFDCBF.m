classdef DCLFDCBF < handle
    properties
        system
        params
        x0
        x_curr
        time_curr = 0.0
        xlog = []
        ulog = []
        u_cost = 0
        obs
    end
    
    methods
        function self = DCLFDCBF(x0, system, params)
            % Define DCLF-DCBF controller
            self.x0 = x0;
            self.x_curr = x0;
            self.system = system;
            self.params = params;
        end
        
        function sim(self, time)
            % Simulate the system until a given time
            xk = self.x_curr;
            while self.time_curr <= time
                % Solve DCLF-DCBF
                uk = self.solveDCLFDCBF();
                xk = self.system.A * xk + self.system.B * uk;
                % update system
                self.x_curr = xk;
                self.time_curr = self.time_curr + self.system.dt;
                self.xlog = [self.xlog, xk];
                self.ulog = [self.ulog, uk];
                self.u_cost = self.u_cost + uk'*uk*self.system.dt;
            end
        end
        
        function uopt = solveDCLFDCBF(self)
            % Solve DCLF-DCBF
            x = self.x_curr;
            u = sdpvar(2,1);
            delta = sdpvar(1,1);
            constraints = [];
            % add DCLF constraints
            x_next = self.system.A * x + self.system.B * u;
            v = x' * self.params.P * x;
            v_next = x_next' * self.params.P * x_next;
            constraints = [constraints; v_next - v + self.params.alpha * v <= delta];
            % add DCBF constraints
            pos = self.obs.pos;
            r = self.obs.r;
            b = (x(1:2)-pos)'*(x(1:2)-pos) - r^2;
            b_next = (x_next(1:2)-pos)'*(x_next(1:2)-pos) - r^2;
            constraints = [constraints; b_next - b + self.params.gamma*b >= 0];
            % input constraints
            constraints = [constraints; self.system.ul <= u <= self.system.uu];
            % cost
            cost = u'*self.params.H*u + delta'*self.params.p*delta;
            % solve optimization
            ops = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, ops);
            if diagnostics.problem == 0
                feas = true;
                uopt = value(u);
            else
                feas = false;
                uopt = [];
            end
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function plot(self,figure_name)
            % Plot simulation
            figure('Renderer', 'painters', 'Position', [0 0 400 400]);
            set(gca,'LooseInset',get(gca,'TightInset'));
            hold on;
            % plot trajectory
            plot(self.xlog(1,:), self.xlog(2,:), 'ko-',...
                'LineWidth', 1.0,'MarkerSize',4);
            % plot obstacle
            pos = self.obs.pos;
            r = self.obs.r;
            th = linspace(0,2*pi*100);
            x = cos(th) ; y = sin(th) ;
            plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);
            plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980], 'MarkerSize', 5, 'LineWidth', 2);
            % plot target position
            plot(self.x0(1), self.x0(2), 'db', 'LineWidth', 1);
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
            print(gcf,strcat('figures/',figure_name), '-depsc');
        end
    end
end