% FILE: tune_objective_LQR_EKF.m

function score = tune_objective_LQR_EKF(p)
    % If no input argument is provided, use default parameter values.
    if nargin < 1
        % Default parameter vector: [Q11, Q22, R, obs_factor]
        p = [100, 0.3, 0.2, 2];
    end

    % Unpack parameters
    Q11 = p(1);
    Q22 = p(2);
    R   = p(3);
    obs_factor = p(4);
    
    % Create the controller instance
    controller = tune_studentControllerInterface_LQR_EKF();
    
    % Set custom properties to override defaults
    controller.custom_Q = diag([Q11, Q22, 0, 0]);  % update Q
    controller.custom_R = R;                        % update R
    controller.custom_obs_factor = obs_factor;      % update observer factor
    
    % Run the simulation using your runner function.
    score = tune_run_matlab_ball_and_beam(controller);
    
    % Optionally display the parameters and resulting score for debugging.
    fprintf('Parameters: Q11=%.2f, Q22=%.2f, R=%.2f, obs_factor=%.2f, Score=%.4f\n',...
            Q11, Q22, R, obs_factor, score);
end
