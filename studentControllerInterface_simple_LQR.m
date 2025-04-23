classdef studentControllerInterface_simple_LQR < matlab.System
    properties (Access = private)
        t_prev = -1;
        theta_d = 0;
        prev_p_ball = 0;
        K = [0 0]; % LQR gain
        initialized = false;
    end

    methods(Access = protected)
        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Constants for simple linearized model
            g = 9.81;
            L = 0.4255;
            rg = 0.0254;

            % Linearized dynamics constants (from linearizing system)
            K1 = 5 * g * rg / (7 * L);  % Linearized acceleration sensitivity to theta
            K2 = 1;  % Theta input gain (treated as 1 since theta is control input)

            % Reference
            [p_ref, v_ref, ~] = get_ref_traj(t);

            % Initialization
            if ~obj.initialized
                % Discretized system (simple Euler with dt=0.01)
                A = [0 1; K1 0];
                B = [0; K2];
                Q = diag([100, 0]);
                R = 8;

                % Continuous LQR
                [K_lqr, ~, ~] = lqr(A, B, Q, R);
                obj.K = K_lqr;
                obj.initialized = true;
                obj.prev_p_ball = p_ball;
            end

            % Estimate velocity via finite difference
            if obj.t_prev < 0
                dt = 1e-3;
            else
                dt = max(t - obj.t_prev, 1e-3);
            end
            v_ball = (p_ball - obj.prev_p_ball) / dt;

            % LQR state vector
            x = [p_ball - p_ref; v_ball - v_ref];

            % Compute control input (desired theta)
            theta_d = -obj.K * x;

            % Clamp angle to hardware limits
            theta_max = 56 * pi / 180;
            theta_d = max(min(theta_d, theta_max), -theta_max);
            obj.theta_d = theta_d;

            % Inner-loop servo PD control
            k_servo = 10;
            V_servo = k_servo * (theta_d - theta);

            % Update internal state
            obj.t_prev = t;
            obj.prev_p_ball = p_ball;
        end
    end

    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
