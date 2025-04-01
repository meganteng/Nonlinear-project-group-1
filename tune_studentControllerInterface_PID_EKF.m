% FILE: tune_studentControllerInterface_PID_EKF.m


classdef tune_studentControllerInterface_PID_EKF < matlab.System
    %% Public properties for autotuning
    properties (Access = public)
        custom_Kp = [];         % Custom proportional gain (if empty, default is 3)
        custom_Ki = [];         % Custom integral gain (if empty, default is 1)
        custom_Kd = [];         % Custom derivative gain (if empty, default is 5)
        custom_obs_factor = []; % Custom EKF process noise scaling factor (if empty, default is 1)
    end

    %% Private properties for internal use
    properties (Access = private)
        % PID gains and states
        Kp = 3;      % Default proportional gain
        Ki = 1;      % Default integral gain
        Kd = 5;      % Default derivative gain
        
        t_prev = -1;         % Previous timestep
        theta_d = 0;         % Desired beam angle computed by PID
        integral_error = 0;  % Accumulated error for integral term
        prev_error = 0;      % Previous error for derivative computation
        
        % EKF variables for state estimation (states: [p_ball; v_ball; theta; dtheta])
        x_hat;  % Estimated state vector
        P;      % Covariance matrix for the EKF
        prev_u = 0;  % Previous control input (servo voltage) used in EKF prediction
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Use custom PID gains if provided.
            if ~isempty(obj.custom_Kp)
                obj.Kp = obj.custom_Kp;
            end
            if ~isempty(obj.custom_Ki)
                obj.Ki = obj.custom_Ki;
            end
            if ~isempty(obj.custom_Kd)
                obj.Kd = obj.custom_Kd;
            end
            % (custom_obs_factor will be used in the EKF prediction.)
            
            % Initialize EKF state estimate and covariance.
            obj.x_hat = zeros(4,1);  % [p_ball; v_ball; theta; dtheta]
            obj.P = 1e-3 * eye(4);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Define system parameters (as used in the ball-and-beam dynamics)
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L_beam = 0.4255;
            
            % Get reference trajectory (assumed available via get_ref_traj).
            [p_ball_ref, ~, ~] = get_ref_traj(t);
            
            % Compute the time step.
            if obj.t_prev < 0
                dt = 1e-3;
            else
                dt = max(t - obj.t_prev, 1e-3);
            end
            
            %% EKF State Estimation
            % Measured outputs: ball position and beam angle.
            y = [p_ball; theta];
            
            % Define the nonlinear dynamics function for prediction.
            % (ball_and_beam_dynamics should be defined elsewhere to return xdot given (t,x,u).)
            f = @(x) ball_and_beam_dynamics(t, x, obj.prev_u);
            
            % Prediction step.
            x_hat_pred = obj.x_hat + f(obj.x_hat) * dt;
            F = obj.computeJacobian(f, obj.x_hat);
            
            % Process noise covariance: scale a nominal value by the custom_obs_factor.
            if isempty(obj.custom_obs_factor)
                obs_factor = 1;
            else
                obs_factor = obj.custom_obs_factor;
            end
            Q_process = obs_factor * 1e-4 * eye(4);
            
            % Propagate the error covariance.
            P_pred = obj.P + (F * obj.P + obj.P * F' + Q_process) * dt;
            
            % Measurement update.
            % The measurement function is h(x) = [x(1); x(3)] (ball position and beam angle).
            y_pred = [x_hat_pred(1); x_hat_pred(3)];
            H = [1 0 0 0; 0 0 1 0];
            R_meas = 1e-2 * eye(2);
            
            innovation = y - y_pred;
            S = H * P_pred * H' + R_meas;
            K_ekf = P_pred * H' / S;
            
            % Update state estimate and covariance.
            obj.x_hat = x_hat_pred + K_ekf * innovation;
            obj.P = (eye(4) - K_ekf * H) * P_pred;
            
            % Extract the filtered ball position and beam angle.
            p_ball_hat = obj.x_hat(1);
            theta_hat  = obj.x_hat(3);
            
            %% PID Computation (using the filtered ball position)
            error = p_ball_hat - p_ball_ref;
            obj.integral_error = obj.integral_error + error * dt;
            derivative_error = (error - obj.prev_error) / dt;
            theta_d = -(obj.Kp * error + obj.Ki * obj.integral_error + obj.Kd * derivative_error);
            
            % Enforce saturation limits on the desired beam angle.
            theta_saturation = 45 * pi / 180;
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);
            
            % Use constant servo gain k_servo = 10.
            k_servo = 10;
            
            % Compute the servo voltage based on the difference between desired and filtered beam angle.
            V_servo = k_servo * (theta_d - theta_hat);
            
            %% Update internal states for next iteration.
            obj.t_prev = t;
            obj.prev_error = error;
            obj.theta_d = theta_d;
            obj.prev_u = V_servo;
        end
    end
    
    methods(Access = public)
        % MATLAB simulation interface.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)    
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
    methods(Access = private)
        function F = computeJacobian(obj, f, x)
            % Numerically compute the Jacobian of f at state x.
            epsilon = 1e-5;
            n = length(x);
            F = zeros(n, n);
            fx = f(x);
            for i = 1:n
                x_eps = x;
                x_eps(i) = x_eps(i) + epsilon;
                F(:, i) = (f(x_eps) - fx) / epsilon;
            end
        end
    end
end