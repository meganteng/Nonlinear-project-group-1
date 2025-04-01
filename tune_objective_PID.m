% FILE: tune_objective_PID.m

function score = tune_objective_PID(p)
    % Parameter vector: [Kp, Ki, Kd]
    if nargin < 1
        p = [3, 1, 5];
    end

    % Unpack parameters.
    Kp = p(1);
    Ki = p(2);
    Kd = p(3);
    
    % Create a controller instance.
    controller = tune_studentControllerInterface_PID();
    
    % Set custom properties for autotuning.
    controller.custom_Kp = Kp;
    controller.custom_Ki = Ki;
    controller.custom_Kd = Kd;
    
    % Run simulation (using your existing simulation runner).
    score = tune_run_matlab_ball_and_beam(controller);
    
    % Display current parameter values and score.
    fprintf('Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, Score=%.4f\n', ...
            Kp, Ki, Kd, score);
end
