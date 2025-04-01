% FILE: tune_objective_PID.m

function score = tune_objective(p)
    % Parameter vector: [Kp, Ki, Kd]
    if nargin < 1
        p = [3, 1, 5];
    end

    % Unpack parameters.
    Kp = p(1);
    Ki = p(2);
    Kd = p(3);
    k_servo = 10;  % Fixed k_servo value

    % Create a controller instance.
    controller = studentControllerInterface();
    
    % Set custom properties for autotuning.
    controller.custom_Kp = Kp;
    controller.custom_Ki = Ki;
    controller.custom_Kd = Kd;
    controller.custom_k_servo = k_servo;
    
    % Run simulation (using your existing simulation runner).
    score = run_matlab_ball_and_beam(controller);
    
    % Display current parameter values and score.
    fprintf('Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, k_servo=%.2f, Score=%.4f\n', ...
            Kp, Ki, Kd, k_servo, score);
end
