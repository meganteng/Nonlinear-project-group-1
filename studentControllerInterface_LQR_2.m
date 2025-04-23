% studentControllerInterface_LQR_2
classdef studentControllerInterface_LQR_2 < matlab.System

    properties (Access = private)
        K = zeros(1, 4);  % LQR gain matrix
        L = zeros(4, 2);  % Luenberger Observer Gains
        t_prev = -1;
        theta_d = 0;
        prev_p_ball = 0;
        prev_theta = 0;
        prev_p_ball_ref = 0;
        prev_t = -1;
        x_hat = zeros(4, 1);
        prev_u = 0;

        A = zeros(4, 4);
        B = zeros(4, 1);
        C = zeros(2, 4);
    end

    methods(Access = protected)
        function setupImpl(obj)
%             coder.extrinsic("lqr");
% 
%             % System constants
%             g = 9.81;
%             tau = 0.025;
%             K_motor = 1.5;
%             rg = 0.0254;
%             L = 0.4255;
% 
%             % Linearized system matrices
%             A = [0 1 0 0;
%                  0 0 5*g*rg/(7*L) 0;
%                  0 0 0 1;
%                  0 0 0 -1/tau];
%             B = [0; 0; 0; K_motor/tau];
% 
%             % Cost weights
%             Q = diag([100, 1, 10, 1]);
%             R = 200;
% 
%             obj.K = zeros(1, 4);
%             obj.K = lqr(A, B, Q, R);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta, Ad, Bd, Cd, K_mx, L_mx)
            for i = 1:4
                obj.K(i) = K_mx(i);
            end
            for i = 1:4
                for j = 1:2
                    obj.L(i, j) = L_mx(i, j);
                end
            end
            for i = 1:4
                for j = 1:4
                    obj.A(i, j) = Ad(i, j);
                end
            end
            for i = 1:4
                obj.B(i) = Bd(i);
            end
            for i = 1:2
                for j = 1:4
                    obj.C(i, j) = Cd(i, j);
                end
            end

            % Parameters
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L = 0.4255;

            % Reference trajectory
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Time delta
            if obj.prev_t < 0
                dt = 0.01;
            else
                dt = max(t - obj.prev_t, 0.01);
            end
            y = [p_ball; theta];
            obj.x_hat = obj.x_hat + (obj.A * obj.x_hat + obj.B * obj.prev_u + obj.L * (y - obj.C * obj.x_hat)) * dt;

            % Finite difference estimation
            % v_ball = (p_ball - obj.prev_p_ball) / dt;
            % v_theta = (theta - obj.prev_theta) / dt;

            % State vector relative to reference
            % p_ball = obj.x_hat(1);
            v_ball = obj.x_hat(2);
            % theta = obj.x_hat(3);
            v_theta = obj.x_hat(4);
            x = [p_ball - p_ball_ref; v_ball - v_ball_ref; theta; v_theta];

            % LQR virtual control (desired third derivative of z)
            v = -obj.K * x + a_ball_ref;

            % Feedback linearization
            a = 5 * g * rg / (7 * L);
            b = (5 * L / 14) * (rg / L)^2;
            c = (5 / 7) * (rg / L)^2;

            cos_theta = cos(theta);
            sin_theta = sin(theta);
            v_theta3 = v_theta^3;

            nonlinear_terms = -b * v_theta3 * cos_theta^2 * sin_theta + ...
                              c * p_ball * v_theta3 * cos_theta^2 * sin_theta;

            dtheta_term = -a * cos_theta * v_theta / tau;
            ddot_theta_coeff = a * cos_theta * K_motor / tau;

            u = (v - nonlinear_terms - dtheta_term) / ddot_theta_coeff;

            % Compute desired angle for logging/debug purposes
            theta_sat = 53 * pi / 180;
            theta_d = asin(min(max( ...
                (7 * L / (5 * g * rg)) * ...
                (v + (5/7) * ((L/2) - p_ball) * (rg/L)^2 * v_theta^2 * cos_theta^2), ...
                -1), 1));
            theta_d = max(min(theta_d, theta_sat), -theta_sat);
            obj.theta_d = theta_d;

            % Final servo voltage
            k_servo = 10;
            V_servo = k_servo * (theta_d - theta);

            % Store for next step
            obj.t_prev = t;
            obj.prev_p_ball = p_ball;
            obj.prev_theta = theta;
            obj.prev_p_ball_ref = p_ball_ref;
            obj.prev_t = t;
            obj.prev_u = u;
        end
    end

    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
