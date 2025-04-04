classdef studentControllerInterface_LQR_Local < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        dt = 0.01;

        % System Parameters.
        rg = 0.0254;       % meters
        L = 0.4255;        % meters
        g = 9.81;
        K = 1.5;
        tau = 0.025;

        % Controller parameters.
        C = [1, 0, 0, 0];
        Q = 1000 * [1, 0, 0, 0]' * [1, 0, 0, 0];
        R = 5;

        % Luenburger Observer parameters.
        x1_hat = 0;       % z (or p_ball) estimator memory buffer.
        x2_hat = 0;       % zdot estimator memory buffer.
        x3_hat = 0;       % theta estimator memory buffer.
        x4_hat = 0;       % theta_dot estimator memory buffer.
        x1_hat_prev = 0;
        x3_hat_prev = 0;
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % This is the main function called every iteration. You have to implement
            % the controller in this function, but you are not allowed to
            % change the signature of this function. 
            % Input arguments:
            %   t: current time
            %   p_ball: position of the ball provided by the ball position sensor (m)
            %
            %   theta: servo motor angle provided by the encoder of the motor (rad)
            % Output:
            %   V_servo: voltage to the servo input.

            % Do not do anything if this is the first step, just record the
            % starting time.
            if obj.t_prev == -1
                obj.t_prev = t;
                obj.x1_hat = p_ball;
                obj.x3_hat = theta;
                V_servo = 0;
                return;
            end

            %% Step sample to get the time derivative.
            obj.x1_hat_prev = obj.x1_hat;
            obj.x3_hat_prev = obj.x3_hat;
            obj.x1_hat = p_ball;
            obj.x2_hat = (obj.x1_hat - obj.x1_hat_prev) / (t - obj.t_prev);
            obj.x3_hat = theta;
            obj.x4_hat = (obj.x3_hat - obj.x3_hat_prev) / (t - obj.t_prev);
            obj.t_prev = t;

            %% LQR Controller.
            [p_ball_ref, v_ball_ref, ~] = get_ref_traj(t);
            [A, B] = obj.linearizeModel(obj.x1_hat, obj.x2_hat, obj.x3_hat, obj.x4_hat);
            [K_lqr, ~, ~] = lqr(A, B, obj.Q, obj.R);
            x_curr = [obj.x1_hat; obj.x2_hat; obj.x3_hat; obj.x4_hat];
            x_ref = [p_ball_ref; v_ball_ref; 0; 0];
            V_servo = -K_lqr * (x_curr - x_ref);

        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)  
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
        
        function [A, B] = linearizeModel(obj, x1, x2, x3, x4)
            A = zeros(4, 4);
            A(1, 2) = 1;
            A(2, 1) = 5/7*(obj.rg/obj.L).^2*x4.^2*(cos(x3)).^2;
            A(2, 3) = 5*obj.g/7*obj.rg/obj.L*cos(x3) + 5/7*(obj.L/2-x1)*(obj.rg/obj.L).^2*x4.^2*(2*cos(x3)*sin(x3));
            A(2, 4) = -5/7*(obj.L/2-x1)*(obj.rg/obj.L).^2*(cos(x3)).^2*(2*x4);
            A(2, 4) = 5/7*(obj.rg/obj.L).^2*x4.^2*(cos(x3)).^2;
            A(3, 4) = 1;
            A(4, 4) = -1/obj.tau;
            B = zeros(4, 1);
            B(4) = obj.K / obj.tau;
        end
    end
    
end
