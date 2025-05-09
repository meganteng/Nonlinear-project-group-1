% _LQR_FL_Luenberger
classdef studentControllerInterface < matlab.System

    properties (Access = private)

        %% Controller Properties
        K = zeros(1, 4);  % LQR Gain Matrix
        L = zeros(4, 2);  % Luenberger Observer Gain Matrix
        A = zeros(4, 4);
        B = zeros(4, 1);
        C = zeros(2, 4);

        %% Internal States
        t_prev = -1;   % Previous timestep
        theta_d = 0;   % Desired beam angle
        prev_p_ball = 0; % Previous ball position for velocity estimation
        prev_theta = 0;  % Previous beam angle for velocity estimation
        prev_p_ball_ref = 0; % Previous reference trajectory position
        prev_t = -1;  % Previous time for finite difference

        %% Observer States
        x_hat = zeros(4, 1);  % Estimated state vector
        prev_u = 0; % Previous control input for observer
    end

    methods(Access = protected)
        function setupImpl(obj)

            % coder.extrinsic("c2d", "lqr", "place");
            % 
            % % Define system parameters
            % g = 9.81;   % Gravity (m/s^2)
            % tau = 0.025; % Motor time constant (s)
            % K_motor = 1.5; % Motor gain (rad/sV)
            % rg = 0.0254; % Servo arm length (m)
            % L_beam = 0.4255; % Beam length (m)
            % 
            % % Define system matrices (A, B, C)
            % Ac = [0 1 0 0;
            %     0 0 5*g*rg/(7*L_beam) 0;
            %     0 0 0 1;
            %     0 0 0 -1/tau];
            % Bc = [0; 0; 0; K_motor/tau];
            % Cc = [1 0 0 0;
            %      0 0 1 0];
            % Q = diag([298, 6.87, 0, 0]); % Fill in your optimal Q matrix here
            % R = 0.406;    % Fill in your optimal R value here
            % dt = 0.01;  % Sampling time
            % 
            % % Discretize A, B using zero-order hold
            % [Ad, Bd] = c2d(Ac, Bc, dt);
            % 
            % % Save these
            % for i = 1:4
            %     for j = 1:4
            %         obj.A(i, j) = Ad(i, j);
            %     end
            % end
            % for i = 1:4
            %     obj.B(i) = Bd(i);
            % end
            % obj.C = Cc;
            % 
            % % LQR gain can still be computed in continuous-time
            % K_mx = lqr(Ac, Bc, Q, R);
            % for i = 1:4
            %     obj.K(i) = K_mx(i);
            % end
            % 
            % % Observer design: place discrete poles (can multiply continuous poles' magnitudes)
            % manual_pole_radius = 0.3;
            % manual_poles = manual_pole_radius * [0.5, 0.5, 0.9, 0.9];
            % L_mx = place(Ad', Cc', manual_poles)';  % Note the transpose
            % for i = 1:4
            %     for j = 1:2
            %         obj.L(i, j) = L_mx(i, j);
            %     end
            % end
            % 
            % % Initialize estimated state
            % obj.x_hat = zeros(4, 1); % [p_ball; v_ball; theta; dtheta]
        end

        function V_servo = stepImpl(obj, t, p_ball, theta, Ad, Bd, Cd, K_mx, L_mx)
            obj.A = Ad;
            obj.B = Bd;
            obj.C = Cd;
            obj.K = zeros(1, 4);
            obj.K(:) = K_mx(:);
            obj.L = zeros(4, 2);
            obj.L(:, :) = L_mx(:, :);

            % Define system parameters
            g = 9.81;
            tau = 0.025;
            K_motor = 1.5;
            rg = 0.0254;
            L_beam = 0.4255;

            % Get reference trajectory
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Luenberger observer update
            % Measured output
            y = [p_ball; theta];

            % % Observer dynamics
            % dx_hat = A * obj.x_hat + B * obj.prev_u + obj.L * (y - C * obj.x_hat);
            % obj.x_hat = obj.x_hat + dx_hat * dt;
            obj.x_hat = obj.A * obj.x_hat + obj.B * obj.prev_u + obj.L * (y - obj.C * obj.x_hat);

            % Extract estimated states
            p_ball_hat = obj.x_hat(1);
            v_ball_hat = obj.x_hat(2);
            theta_hat = obj.x_hat(3);
            dtheta_hat = obj.x_hat(4);

            % Construct the state vector with estimated velocities
            x = [p_ball_hat - p_ball_ref; v_ball_hat - v_ball_ref; theta_hat; dtheta_hat];

            % Compute optimal control input using LQR (virtual input)
            v = -obj.K * x + a_ball_ref;

            % Feedback linearization: Compute the control input u
            a = 5 * g * rg / (7 * L_beam);
            b = (5 * L_beam / 14) * (rg / L_beam)^2;
            c = (5 / 7) * (rg / L_beam)^2;

            % Nonlinear terms in the third derivative
            nonlinear_terms = a * cos(theta_hat) * dtheta_hat ...
                - b * (2 * dtheta_hat^3 * cos(theta_hat)^2 * sin(theta_hat)) ...
                + c * p_ball_hat * (2 * dtheta_hat^3 * cos(theta_hat)^2 * sin(theta_hat));

            % Solve for u to achieve dddy = v
            u = (v - nonlinear_terms) * tau / (a * cos(theta_hat) * K_motor);

            % Apply physical constraints
            theta_saturation = 45 * pi / 180; % Limit servo movement to ±45 degrees
            theta_d = asin(min(max(...
                (7 * L_beam / (5 * g * rg)) * (v + (5/7) * ((L_beam/2) - p_ball_hat) * (rg/L_beam)^2 * dtheta_hat^2 * cos(theta_hat)^2), ...
                -1), 1)); % Ensure valid range for asin
            theta_d = max(min(theta_d, theta_saturation), -theta_saturation);

            % Compute servo voltage
            k_servo = 10;  % Servo gain
            V_servo = k_servo * (theta_d - theta_hat);

            % Store values for next iteration
            obj.t_prev = t;
            obj.theta_d = theta_d;
            obj.prev_p_ball = p_ball;
            obj.prev_theta = theta;
            obj.prev_p_ball_ref = p_ball_ref;
            obj.prev_t = t;
            obj.prev_u = u; % Store control input for observer
        end
    end

    methods(Access = public)
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta, Ad, Bd, Cd, K_mx, L_mx)
            V_servo = stepImpl(obj, t, p_ball, theta, Ad, Bd, Cd, K_mx, L_mx);
            theta_d = obj.theta_d;
        end

        % Public method to access the estimated states
        function x_hat = getObserverStates(obj)
            x_hat = obj.x_hat;
        end
    end
end