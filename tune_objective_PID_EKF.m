% FILE: tune_objective_PID_EKF.m

function score = tune_objective_PID_EKF(p)
    % If no input argument is provided, use default parameter values.
    if nargin < 1
        % Default parameter vector: [Kp, Ki, Kd, obs_factor]
        p = [3, 1, 5, 1];
    end

    % Unpack parameters
    Kp = p(1);
    Ki = p(2);
    Kd = p(3);
    obs_factor = p(4);
    
    % Create the controller instance (PID+EKF version)
    controller = tune_studentControllerInterface_PID_EKF();
    
    % Set custom properties to override defaults
    controller.custom_Kp = Kp;      
    controller.custom_Ki = Ki;      
    controller.custom_Kd = Kd;      
    controller.custom_obs_factor = obs_factor;  % EKF process noise scaling factor
    
    % Run the simulation using your runner function.
    score = tune_run_matlab_ball_and_beam(controller);
    
    % Optionally display the parameters and resulting score for debugging.
    fprintf('Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, obs_factor=%.2f, Score=%.4f\n',...
            Kp, Ki, Kd, obs_factor, score);
end
