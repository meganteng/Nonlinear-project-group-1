% _LQR_full_feedback
classdef studentControllerInterface_LQR_full_feedback < matlab.System

    properties (Access = private)

        %% Internal States
        t_prev = -1;   % Previous timestep
        theta_d = 0;   % Desired beam angle
        prev_p_ball = 0; % Previous ball position for velocity estimation
        prev_theta = 0;  % Previous beam angle for velocity estimation
        prev_p_ball_ref = 0; % Previous reference trajectory position
        prev_t = -1;  % Previous time for finite difference
    end
    
    methods(Access = protected)
        function setupImpl(obj)
% 
%             coder.extrinsic("lqr");
% 
%             % Define system parameters
%             g = 9.81;   % Gravity (m/s^2)
%             tau = 0.025; % Motor time constant (s)
%             K_motor = 1.5; % Motor gain (rad/sV)
%             rg = 0.0254; % Servo arm length (m)
%             L = 0.4255; % Beam length (m)
% 
%             % Define updated A, B matrices with tau included
%             A = [0 1 0 0; 
%                  0 0 5*g*rg/(7*L) 0; 
%                  0 0 0 1; 
%                  0 0 0 -1/tau];
% 
%             B = [0; 0; 0; K_motor/tau];
% 
%             % Define LQR weight matrices
%             Q = diag([100, 0.3, 0, 0]); % Adjusted Q for smoother control
%             R = 0.2;  % Increased control effort penalty
% 
%             % Compute LQR gain
%             K_mx = lqr(A, B, Q, R);
%             for i = 1:4
%                 obj.K(i) = K_mx(i);
%             end

        end

        function V_servo = stepImpl(obj, t, p_ball, theta, K_mx)
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
                dt = 1e-3; % Avoid division by zero on first call
            else
                dt = max(t - obj.prev_t, 1e-3);
            end

            % Estimate velocities using finite difference
            v_ball = (p_ball - obj.prev_p_ball) / dt;
            v_theta = (theta - obj.prev_theta) / dt;
            v_ball_ref = (p_ball_ref - obj.prev_p_ball_ref) / dt; % Reference velocity

            % Construct the state vector with estimated velocities
            x = [p_ball - p_ball_ref; v_ball - v_ball_ref; theta; v_theta];

            % Compute optimal control input using LQR (virtual input)
            v = -K_mx * x + a_ball_ref;

            % Feedback linearization: Compute the control input u
            % Third derivative of the output (ball position)
            % dddy = v (desired third derivative from LQR)
            % We need to solve for u in the equation dddy = v

            % Compute intermediate terms
            a = 5 * g * rg / (7 * L);
            b = (5 * L / 14) * (rg / L)^2;
            c = (5 / 7) * (rg / L)^2;

            % First derivative of theta (already estimated as v_theta)
            dtheta = v_theta;

            % Nonlinear terms in the third derivative
            nonlinear_terms = a * cos(theta) * dtheta ...
                - b * (2 * dtheta^3 * cos(theta)^2 * sin(theta)) ...
                + c * p_ball * (2 * dtheta^3 * cos(theta)^2 * sin(theta));

            % Solve for u to achieve dddy = v
            u = (v - nonlinear_terms) * tau / (a * cos(theta) * K_motor);

            % Apply physical constraints
            theta_saturation = 45 * pi / 180; % Limit servo movement to ±45 degrees
            theta_d = asin(min(max(...
                (7 * L / (5 * g * rg)) * (v + (5/7) * ((L/2) - p_ball) * (rg/L)^2 * v_theta^2 * cos(theta)^2), ...
                -1), 1)); % Ensure valid range for asin
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Compute servo voltage
            k_servo = 10;  % Servo gain
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
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta, K_mx)    
            V_servo = stepImpl(obj, t, p_ball, theta, K_mx);
            theta_d = obj.theta_d;
        end
    end
end