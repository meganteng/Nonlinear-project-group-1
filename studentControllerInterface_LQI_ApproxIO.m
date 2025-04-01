% _LQI_ApproxIO
% Linear Quadratic Integral controller with Approximate I/O Linearization
classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% Controller Parameters
        K;          % LQR Gain Matrix
        Ki;         % Integral Gain
        
        %% Internal States
        t_prev = -1;         % Previous timestep
        theta_d = 0;         % Desired beam angle
        prev_p_ball = 0;     % previous ball position
        prev_theta = 0;      % previous beam angle
        prev_p_ball_ref = 0; % previous reference trajectory position
        prev_t = -1;         % previous time-step for finite difference
        integral_error = 0;  % integral of position error
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Define system parameters (ensure these match across simulation/hardware)
            g = 9.81;           % Gravity (m/s^2)
            tau = 0.025;        % Motor time constant (s)
            K_motor = 1.5;      % Motor gain (rad/sV)
            rg = 0.0254;        % Servo arm length (m)
            L_beam = 0.4255;    % Beam length (m)

            % Define linearized system matrices
            % Approximate I/O linearization by disregarding small nonlinear terms
            % Resulting in a system with a well-defined relative degree
            
            % Linearized state-space model
            A = [0 1 0 0; 
                 0 0 5*g*rg/(7*L_beam) 0; 
                 0 0 0 1; 
                 0 0 0 -1/tau];

            B = [0; 0; 0; K_motor/tau];
            
            % Augmented state-space for integral action
            % Add an integrator state for the tracking error
            A_aug = [A, zeros(4,1); 
                     1, 0, 0, 0, 0]; % Integral of position error
            
            B_aug = [B; 0];
            
            % Define LQR weight matrices
            % Balance between tracking performance and control effort
            Q = diag([100, 1, 0.1, 0.1, 50]); % Last element weights the integral action
            R = 0.2;  % Control effort penalty
            
            % Compute LQI gain matrix [K Ki]
            K_aug = lqr(A_aug, B_aug, Q, R);
            
            % Extract the state feedback and integral gains
            obj.K = K_aug(1:4);
            obj.Ki = K_aug(5);
            
            % Initialize integral error
            obj.integral_error = 0;
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Define system parameters (ensure consistency with setupImpl)
            g = 9.81;   
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254; 
            L_beam = 0.4255;

            % Get reference trajectory
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Compute time step
            if obj.prev_t == -1
                dt = 1e-3; % Avoid division by zero on first call
            else
                dt = max(t - obj.prev_t, 1e-3);
            end

            % Estimate velocities using finite difference
            v_ball = (p_ball - obj.prev_p_ball) / dt;
            v_theta = (theta - obj.prev_theta) / dt;
            
            % Calculate position error and update integral term
            pos_error = p_ball - p_ball_ref;
            obj.integral_error = obj.integral_error + pos_error * dt;
            
            % Anti-windup for integral term (prevent excessive accumulation)
            max_integral = 0.5; % Tunable parameter
            obj.integral_error = max(min(obj.integral_error, max_integral), -max_integral);
            
            % Construct state vector
            x = [p_ball - p_ball_ref; 
                 v_ball - v_ball_ref; 
                 theta; 
                 v_theta];
            
            % Compute control input using LQI
            u_lqi = -obj.K * x - obj.Ki * obj.integral_error + a_ball_ref;
            
            % Approximate I/O linearization
            % This simplifies the full feedback linearization by
            % approximating the nonlinear dynamics of the ball-beam system
            
            % Convert desired acceleration to desired beam angle using simplified model
            % Linearized relationship between ball acceleration and beam angle
            a = 5 * g * rg / (7 * L_beam);
            
            % Approximate linearization omitting higher-order nonlinear terms
            % Based on the description, we disregard small nonlinear terms
            theta_d = asin(min(max(u_lqi / a, -1), 1)); 
            
            % Apply physical constraints
            theta_saturation = 45 * pi / 180; % Limit servo movement to ±45 degrees
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Compute servo voltage using a servo gain
            k_servo = 10;  % Servo gain (may need tuning in hardware implementation)
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
        
        % Public method to reset integral term if needed
        function resetIntegral(obj)
            obj.integral_error = 0;
        end
    end
end
