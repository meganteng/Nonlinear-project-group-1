% FILE: studentControllerInterface_LQR_FL_Luenberger_extra_params.m 

classdef studentControllerInterface < matlab.System

    %% New public properties for tuning
    properties (Access = public)
        custom_Q = [];         % Custom Q matrix, now can be 4x4 (if empty, use default)
        custom_R = [];         % Custom R value (if empty, use default)
        custom_obs_factor = []; % Custom observer factor (if empty, use default 2)
        custom_nl_scale = [];  % Custom scaling for nonlinear cancellation (default = 1)
    end

    properties (Access = private)
        %% Controller Properties
        K; % LQR Gain Matrix
        L; % Luenberger Observer Gain Matrix

        %% Internal States
        t_prev = -1;
        theta_d = 0;
        prev_p_ball = 0;
        prev_theta = 0;
        prev_p_ball_ref = 0;
        prev_t = -1;

        %% Observer States
        x_hat;
        prev_u = 0;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % System parameters
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L_beam = 0.4255;

            % System matrices
            A = [0 1 0 0;
                 0 0 5*g*rg/(7*L_beam) 0;
                 0 0 0 1;
                 0 0 0 -1/tau];
            B = [0; 0; 0; K_motor/tau];
            C = [1 0 0 0;
                 0 0 1 0];

            % Use custom Q/R if provided; otherwise, default.
            if isempty(obj.custom_Q)
                % Default: penalize ball position and velocity only.
                Q = diag([100, 0.3, 0, 0]);
            else
                Q = obj.custom_Q;  % Expected to be a 4x4 diagonal matrix.
            end
            if isempty(obj.custom_R)
                R = 0.2;
            else
                R = obj.custom_R;
            end

            % Compute LQR gain.
            obj.K = lqr(A, B, Q, R);

            % Determine observer factor.
            if isempty(obj.custom_obs_factor)
                obs_factor = 2;
            else
                obs_factor = obj.custom_obs_factor;
            end

            % Design observer gain.
            observer_poles = obs_factor * eig(A - B * obj.K);
            obj.L = place(A', C', observer_poles)';

            % Initialize estimated state.
            obj.x_hat = zeros(4, 1);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % System parameters
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L_beam = 0.4255;

            % Get reference trajectory.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Time step for velocity estimation.
            if obj.prev_t == -1
                dt = 1e-3;
                obj.x_hat = [p_ball; 0; theta; 0];
            else
                dt = max(t - obj.prev_t, 1e-3);
            end

            % Observer update.
            y = [p_ball; theta];
            A = [0 1 0 0;
                 0 0 5*g*rg/(7*L_beam) 0;
                 0 0 0 1;
                 0 0 0 -1/tau];
            B = [0; 0; 0; K_motor/tau];
            C = [1 0 0 0;
                 0 0 1 0];
            dx_hat = A * obj.x_hat + B * obj.prev_u + obj.L * (y - C * obj.x_hat);
            obj.x_hat = obj.x_hat + dx_hat * dt;

            p_ball_hat = obj.x_hat(1);
            v_ball_hat = obj.x_hat(2);
            if obj.prev_t == -1
                measured_dtheta = 0;
            else
                measured_dtheta = (theta - obj.prev_theta) / dt;
            end

            % Construct state for LQR.
            x = [p_ball_hat - p_ball_ref; v_ball_hat - v_ball_ref; theta; measured_dtheta];

            % Virtual control input.
            v = -obj.K * x + a_ball_ref;

            % Feedback linearization.
            a = 5 * g * rg / (7 * L_beam);
            b = (5 * L_beam / 14) * (rg / L_beam)^2;
            c = (5 / 7) * (rg / L_beam)^2;
            nonlinear_terms = a * cos(theta) * measured_dtheta ...
                - b * (2 * measured_dtheta^3 * cos(theta)^2 * sin(theta)) ...
                + c * p_ball_hat * (2 * measured_dtheta^3 * cos(theta)^2 * sin(theta));
            
            % Scale nonlinear cancellation if custom value provided.
            if isempty(obj.custom_nl_scale)
                nl_scale = 1;
            else
                nl_scale = obj.custom_nl_scale;
            end

            u = (v - nl_scale * nonlinear_terms) * tau / (a * cos(theta) * K_motor);

            % Compute desired beam angle and servo voltage.
            theta_saturation = 45 * pi / 180;
            theta_d = asin( min( max( (7 * L_beam / (5 * g * rg)) * ...
                (v + (5/7) * ((L_beam/2) - p_ball_hat) * (rg/L_beam)^2 * measured_dtheta^2 * cos(theta)^2), -1), 1) );
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);
            
            % Fixed servo gain constant
            k_servo = 10;

            V_servo = k_servo * (theta_d - theta);

            % Store values.
            obj.t_prev = t;
            obj.theta_d = theta_d;
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

        function x_hat = getObserverStates(obj)
            x_hat = obj.x_hat;
        end
    end

end
