% PID
classdef studentControllerInterface_PID < matlab.System
    properties (Access = private)
        %% Controller Parameters
        Kp = 3;  % Proportional gain
        Ki = 1;  % Integral gain
        Kd = 5;  % Derivative gain
        
        %% Internal States
        t_prev = -1;   % Previous timestep
        theta_d = 0;   % Desired beam angle
        integral_error = 0; % Integral term for PID
        prev_error = 0; % Previous error for derivative term
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Setup function (not needed for PID, but left here for structure)
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
        % This function is called every iteration and implements the PID control.
        % Input:
        %   t: current time
        %   p_ball: position of the ball (m)
        %   theta: servo motor angle (rad)
        % Output:
        %   V_servo: voltage command for servo motor
        
            % Extract reference trajectory
            [p_ball_ref, ~, ~] = get_ref_traj(t);

            % Compute position error
            error = p_ball - p_ball_ref;

            % Compute time step
            dt = max(t - obj.t_prev, 1e-3); % Prevent division by zero

            % PID Control Computation
            obj.integral_error = obj.integral_error + error * dt;  % Accumulate integral
            derivative_error = (error - obj.prev_error) / dt;  % Compute derivative

            % Compute desired servo angle using PID
            theta_d = -(obj.Kp * error + obj.Ki * obj.integral_error + obj.Kd * derivative_error);

            % Apply physical limits on servo angle
            theta_saturation = 45 * pi / 180; % Prevent excessive tilting
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Compute servo voltage
            k_servo = 5;  % Servo gain
            V_servo = k_servo * (theta_d - theta);
            
            % Update stored values for next iteration
            obj.t_prev = t;
            obj.prev_error = error;
            obj.theta_d = theta_d;
        end
    end
    
    methods(Access = public)
        % MATLAB simulation interface
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)    
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
