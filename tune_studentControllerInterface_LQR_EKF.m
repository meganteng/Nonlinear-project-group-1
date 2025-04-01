% FILE: tune_studentControllerInterface_LQR_EKF.m 

classdef studentControllerInterface < matlab.System
    % Controller using LQR with EKF observer for state estimation

    properties
        custom_Q;             % Custom LQR Q matrix (if provided)
        custom_R;             % Custom LQR R (if provided)
        custom_obs_factor;    % Factor to scale process noise covariance in the EKF
    end

    properties(Access = private)
        %% Controller Properties
        K;  % LQR Gain Matrix
        % (Note: the Luenberger gain L is no longer needed)

        %% EKF Variables
        P;  % Covariance matrix for the EKF
        
        %% Internal States
        t_prev = -1;   % Previous timestep
        theta_d = 0;   % Desired beam angle
        prev_p_ball = 0; % Previous ball position for velocity estimation
        prev_theta = 0;  % Previous beam angle for velocity estimation
        prev_p_ball_ref = 0; % Previous reference trajectory position
        prev_t = -1;  % Previous time for finite difference
        
        %% Observer States
        x_hat;  % Estimated state vector
        prev_u = 0; % Previous control input (used in prediction step)
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Define system parameters
            g = 9.81;       % Gravity (m/s^2)
            tau = 0.025;    % Motor time constant (s)
            K_motor = 1.5;  % Motor gain (rad/sV)
            rg = 0.0254;    % Servo arm length (m)
            L_beam = 0.4255; % Beam length (m)

            % Define system matrices (A, B, C)
            A = [0 1 0 0; 
                 0 0 5*g*rg/(7*L_beam) 0; 
                 0 0 0 1; 
                 0 0 0 -1/tau];

            B = [0; 0; 0; K_motor/tau];

            C = [1 0 0 0; 
                 0 0 1 0];

            % Use custom Q and R for LQR if provided, otherwise use defaults.
            if ~isempty(obj.custom_Q)
                Q = obj.custom_Q;
            else
                Q = diag([100, 0.3, 0, 0]);
            end

            if ~isempty(obj.custom_R)
                R = obj.custom_R;
            else
                R = 0.2;
            end

            % Compute LQR gain
            obj.K = lqr(A, B, Q, R);

            % Initialize EKF state estimate (state: [p_ball; v_ball; theta; dtheta])
            obj.x_hat = zeros(4, 1);

            % Initialize the EKF covariance matrix with a small value.
            obj.P = 1e-3 * eye(4);
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % Define system parameters
            g = 9.81;   
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254; 
            L_beam = 0.4255;

            % Get reference trajectory
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Compute time step (dt) for the observer update.
            if obj.prev_t == -1
                dt = 1e-3;
            else
                dt = max(t - obj.prev_t, 1e-3);
            end

            % Measured output (ball position and beam angle)
            y = [p_ball; theta];

            %% EKF Prediction Step
            % Define the nonlinear dynamics function.
            % ball_and_beam_dynamics should be defined elsewhere.
            f = @(x) ball_and_beam_dynamics(t, x, obj.prev_u);
            % Predict state
            x_hat_pred = obj.x_hat + f(obj.x_hat) * dt;

            % Compute the Jacobian F of f with respect to x using numerical differentiation.
            F = obj.computeJacobian(f, obj.x_hat);

            % Define process noise covariance for the EKF.
            if isempty(obj.custom_obs_factor)
                Q_process = 1e-4 * eye(4);
            else
                Q_process = obj.custom_obs_factor * 1e-4 * eye(4);
            end

            % Propagate covariance (discrete-time approximation)
            P_pred = obj.P + (F * obj.P + obj.P * F' + Q_process) * dt;

            %% EKF Update Step
            % Measurement prediction using h(x) = [p_ball; theta] = [x(1); x(3)]
            y_pred = [x_hat_pred(1); x_hat_pred(3)];
            
            % Measurement Jacobian (since h is linear)
            H = [1 0 0 0; 0 0 1 0];

            % Define measurement noise covariance.
            if isempty(obj.custom_R)
                R_meas = 1e-2 * eye(2);
            else
                if isscalar(obj.custom_R)
                    R_meas = obj.custom_R * eye(2);
                else
                    R_meas = diag(obj.custom_R);
                end
            end

            % Calculate the Kalman gain
            K_EKF = P_pred * H' / (H * P_pred * H' + R_meas);
            
            % Update the state estimate
            obj.x_hat = x_hat_pred + K_EKF * (y - y_pred);
            
            % Update the covariance matrix
            obj.P = (eye(4) - K_EKF * H) * P_pred;
            
            % Control law using the estimated state
            V_servo = -obj.K * (obj.x_hat - [p_ball_ref; v_ball_ref; 0; 0]);
            
            % Save the current control input for the next prediction step
            obj.prev_u = V_servo;
            
            % Save current time for the next update
            obj.prev_t = t;
        end
        
        function J = computeJacobian(obj, f, x)
            % Compute the Jacobian matrix for function f at point x
            % using numerical differentiation
            
            n = length(x);
            J = zeros(n, n);
            h = 1e-6;  % Small perturbation
            
            for i = 1:n
                x_perturbed = x;
                x_perturbed(i) = x_perturbed(i) + h;
                J(:, i) = (f(x_perturbed) - f(x)) / h;
            end
        end
    end
    
    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)    
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end

        % Public method to access the estimated states
        function x_hat = getObserverStates(obj)
            x_hat = obj.x_hat;
        end
    end
end
