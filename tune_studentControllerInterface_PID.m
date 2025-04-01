% FILE: tune_studentControllerInterface_PID.m 

classdef studentControllerInterface < matlab.System
    %% Public properties for autotuning
    properties (Access = public)
        custom_Kp = [];       % Custom proportional gain (if empty, default is 3)
        custom_Ki = [];       % Custom integral gain (if empty, default is 1)
        custom_Kd = [];       % Custom derivative gain (if empty, default is 5)
    end

    %% Private properties for internal use
    properties (Access = private)
        Kp = 3;  % Default proportional gain
        Ki = 1;  % Default integral gain
        Kd = 5;  % Default derivative gain
        
        t_prev = -1;         % Previous timestep
        theta_d = 0;         % Desired beam angle
        integral_error = 0;  % Accumulated error for integral term
        prev_error = 0;      % Previous error for derivative computation
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Use custom values if provided.
            if ~isempty(obj.custom_Kp)
                obj.Kp = obj.custom_Kp;
            end
            if ~isempty(obj.custom_Ki)
                obj.Ki = obj.custom_Ki;
            end
            if ~isempty(obj.custom_Kd)
                obj.Kd = obj.custom_Kd;
            end
            % (No setup needed for this simple PID.)
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Get reference trajectory.
            [p_ball_ref, ~, ~] = get_ref_traj(t);

            % Compute the error.
            error = p_ball - p_ball_ref;
            
            % Compute the time step.
            dt = max(t - obj.t_prev, 1e-3); % Avoid division by zero
            
            % PID computation.
            obj.integral_error = obj.integral_error + error * dt;
            derivative_error = (error - obj.prev_error) / dt;
            
            % Calculate desired servo angle.
            theta_d = -(obj.Kp * error + obj.Ki * obj.integral_error + obj.Kd * derivative_error);
            
            % Limit the desired angle.
            theta_saturation = 45 * pi / 180;
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);
            
            k_servo = 10;  % Default servo gain for PID
            
            % Compute servo voltage.
            V_servo = k_servo * (theta_d - theta);
            
            % Update stored values.
            obj.t_prev = t;
            obj.prev_error = error;
            obj.theta_d = theta_d;
        end
    end
    
    methods(Access = public)
        % MATLAB simulation interface.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)    
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
