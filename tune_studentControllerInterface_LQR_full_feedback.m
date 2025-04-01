% FILE: tune_studentControllerInterface_LQR_full_feedback.m 

classdef studentControllerInterface < matlab.System
    %% New public properties for tuning
    properties (Access = public)
        custom_Q = [];         % Custom Q matrix (4x4). If empty, use default diag([100, 0.3, 0, 0])
        custom_R = [];         % Custom R value. If empty, use default 0.2
        custom_k_servo = [];   % Custom servo gain. If empty, use default 10.
        custom_nl_scale = [];  % Custom scaling for nonlinear cancellation (default = 1)
    end

    properties (Access = private)
        %% Controller Properties
        K;  % LQR Gain Matrix
        
        %% Internal States
        t_prev = -1;       % Previous timestep
        theta_d = 0;       % Desired beam angle
        prev_p_ball = 0;   % Previous ball position for velocity estimation
        prev_theta = 0;    % Previous beam angle for velocity estimation
        prev_p_ball_ref = 0; % Previous reference trajectory position
        prev_t = -1;       % Previous time for finite difference
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Define system parameters
            g = 9.81;       % Gravity (m/s^2)
            tau = 0.025;    % Motor time constant (s)
            K_motor = 1.5;  % Motor gain (rad/sV)
            rg = 0.0254;    % Servo arm length (m)
            L = 0.4255;     % Beam length (m)

            % Define updated A, B matrices (including tau)
            A = [0 1 0 0; 
                 0 0 5*g*rg/(7*L) 0; 
                 0 0 0 1; 
                 0 0 0 -1/tau];
            B = [0; 0; 0; K_motor/tau];

            % Use custom Q/R if provided, otherwise defaults:
            if isempty(obj.custom_Q)
                Q = diag([100, 0.3, 0, 0]);
            else
                Q = obj.custom_Q;
            end
            if isempty(obj.custom_R)
                R = 0.2;
            else
                R = obj.custom_R;
            end

            % Compute LQR gain
            obj.K = lqr(A, B, Q, R);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Define system parameters
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L = 0.4255;

            % Get reference trajectory
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Compute time step for velocity estimation
            if obj.prev_t == -1
                dt = 1e-3;  % Avoid division by zero on first call
            else
                dt = max(t - obj.prev_t, 1e-3);
            end

            % Estimate velocities using finite differences
            v_ball = (p_ball - obj.prev_p_ball) / dt;
            v_theta = (theta - obj.prev_theta) / dt;
            v_ball_ref = (p_ball_ref - obj.prev_p_ball_ref) / dt; % Reference velocity

            % Construct the state vector
            x = [p_ball - p_ball_ref; v_ball - v_ball_ref; theta; v_theta];

            % Compute the virtual control input using LQR
            v = -obj.K * x + a_ball_ref;

            % Feedback linearization: compute nonlinear cancellation terms
            a = 5 * g * rg / (7 * L);
            b = (5 * L / 14) * (rg / L)^2;
            c = (5 / 7) * (rg / L)^2;
            dtheta = v_theta;  % Already estimated
            nonlinear_terms = a * cos(theta) * dtheta ...
                - b * (2 * dtheta^3 * cos(theta)^2 * sin(theta)) ...
                + c * p_ball * (2 * dtheta^3 * cos(theta)^2 * sin(theta));
            
            % Apply custom nonlinear scaling if provided
            if isempty(obj.custom_nl_scale)
                nl_scale = 1;
            else
                nl_scale = obj.custom_nl_scale;
            end

            % Solve for u so that dddy = v
            u = (v - nl_scale * nonlinear_terms) * tau / (a * cos(theta) * K_motor);

            % Apply physical constraints on beam angle
            theta_saturation = 45 * pi / 180;
            theta_d = asin( min( max( (7 * L / (5 * g * rg)) * ...
                (v + (5/7) * ((L/2) - p_ball) * (rg/L)^2 * v_theta^2 * cos(theta)^2), -1), 1) );
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Use custom servo gain if provided
            if isempty(obj.custom_k_servo)
                k_servo = 10;
            else
                k_servo = obj.custom_k_servo;
            end

            % Compute servo voltage command
            V_servo = k_servo * (theta_d - theta);
            
            % Store values for next iteration
            obj.t_prev = t;
            obj.theta_d = theta_d;
            obj.prev_p_ball = p_ball;
            obj.prev_theta = theta;
            obj.prev_p_ball_ref = p_ball_ref;
            obj.prev_t = t;
        end
    end
    
    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
