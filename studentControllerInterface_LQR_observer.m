classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% Controller Properties
        K;  % LQR Gain Matrix
        L;  % Observer Gain Matrix
        
        %% Internal States
        t_prev = -1;   % Previous timestep
        theta_d = 0;   % Desired beam angle
        x_hat;  % Estimated state vector [p_ball; v_ball; theta; v_theta]
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Define system parameters
            g = 9.81;   % Gravity (m/s^2)
            tau = 0.025; % Motor time constant (s)
            K_motor = 1.5; % Motor gain (rad/sV)
            rg = 0.0254; % Servo arm length (m)
            L = 0.4255; % Beam length (m)

            % Define system matrices
            A = [0 1 0 0; 
                 0 0 5*g*rg/(7*L) 0; 
                 0 0 0 1; 
                 0 0 0 -1/tau];

            B = [0; 0; 0; K_motor/tau];

            C = [1 0 0 0; % Measurement: Ball position
                 0 0 1 0]; % Measurement: Beam angle

            % Define LQR weight matrices
            Q = diag([10, 0.1, 0.1, 0]); % Penalizing position, velocity, angle, angular velocity
            R = 0.05;  % Penalizing control effort

            % Compute LQR gain
            obj.K = lqr(A, B, Q, R);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Define system parameters
            A = [0 1 0 0; 
                 0 0 5*g*rg/(7*L) 0; 
                 0 0 0 1; 
                 0 0 0 -1/tau];
            g = 9.81;   
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254; 
            L = 0.4255;

            % Get reference trajectory
            [p_ball_ref, ~, a_ball_ref] = get_ref_traj(t);

            % Compute time step for observer update
            if obj.t_prev == -1
                dt = 1e-3; % Avoid division by zero on first call
            else
                dt = max(t - obj.t_prev, 1e-3);
            end
            % Design observer (Luenberger)
            observer_poles = [-10, -11, -12, -13]; % Choose observer poles
            obj.L = place(A', C', observer_poles)'; % Observer gain matrix

            % Initialize state estimate
            obj.x_hat = zeros(4,1);

            % Observer update (Estimate missing velocity states)
            y = [p_ball; theta]; % Measurements
            obj.x_hat = obj.x_hat + obj.L * (y - obj.x_hat([1,3])); % Luenberger update
            x = obj.x_hat; % Estimated state

            % Compute optimal control input using LQR
            v = -obj.K * x + a_ball_ref;

            % Compute desired beam angle using inverse transformation
            theta_d = asin(min(max(...  
                (7 * L / (5 * g * rg)) * (v + (5/7) * ((L/2) - x(1)) * (rg/L)^2 * x(4)^2 * cos(x(3))^2), ...
                -1), 1)); % Ensure valid range for asin
            
            % Apply physical constraints
            theta_saturation = 45 * pi / 180; % Limit servo movement to ±45 degrees
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Compute servo voltage
            k_servo = 10;  % Servo gain
            V_servo = k_servo * (theta_d - theta);
            
            % Store values for next iteration
            obj.t_prev = t;
            obj.theta_d = theta_d;
        end
    end
    
    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)    
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
end
