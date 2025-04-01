% FILE: tune_objective_LQR_full_feedback.m
function score = tune_objective_LQR_full_feedback(p)
    % Use default parameters if none provided.
    % Parameter vector: [Q1, Q2, R, nl_scale]
    if nargin < 1
        p = [100, 0.3, 0.2, 1];
    end

    % Unpack parameters.
    Q1 = p(1);
    Q2 = p(2);
    R   = p(3);
    nl_scale = p(4);

    % Create the controller instance.
    controller = tune_studentControllerInterface_LQR_full_feedback();

    % Set custom properties.
    controller.custom_Q = diag([Q1, Q2, 0, 0]); % Q3 and Q4 are fixed at zero
    controller.custom_R = R;
    controller.custom_nl_scale = nl_scale;
    % k_servo is no longer a tuning parameter, it's fixed at 10 in the controller

    % Run the simulation (using your existing simulation runner).
    score = tune_run_matlab_ball_and_beam(controller);
    fprintf('Parameters: Q=[%.2f, %.2f, 0.00, 0.00], R=%.2f, nl_scale=%.2f, Score=%.4f\n',...
            Q1, Q2, R, nl_scale, score);
end
