% FILE: tune_objective_LFL_Luen_fixObs.m

function score = tune_objective_LFL_Luen_fixObs(p)
    % If no input argument is provided, use default parameter values.
    if nargin < 1
        % Default parameter vector: [Q11, Q22, R]
        p = [100, 0.3, 0.2];
    end

    % Unpack parameters
    Q11 = p(1);
    Q22 = p(2);
    R   = p(3);
    
    % Create the controller instance
    controller = tune_studentControllerInterface_LFL_Luen_fixObs();
    
    % Set custom properties to override defaults
    controller.custom_Q = diag([Q11, Q22, 0, 0]);  
    controller.custom_R = R;                        
    % k_servo is no longer a tuning parameter, it's fixed at 10 in the controller
    
    % Run the simulation using your runner function.
    score = tune_run_matlab_ball_and_beam(controller);
    
    % Optionally display the parameters and resulting score for debugging.
    fprintf('Parameters: Q11=%.8f, Q22=%.8f, R=%.8f, Score=%.8f\n',...
            Q11, Q22, R, score);
end
