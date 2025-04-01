% FILE: tune_objective.m
function score = tune_objective(p)
    % Use default parameters if none provided.
    if nargin < 1
        % Default vector: [Q1, Q2, Q3, Q4, R, obs_factor, k_servo, nl_scale]
        p = [100, 0.3, 0, 0, 0.2, 2, 10, 1];
    end

    % Unpack parameters.
    Q1 = p(1);
    Q2 = p(2);
    Q3 = p(3);
    Q4 = p(4);
    R   = p(5);
    obs_factor = p(6);
    k_servo    = p(7);
    nl_scale   = p(8);

    % Create the controller instance.
    controller = studentControllerInterface();

    % Set custom properties.
    controller.custom_Q = diag([Q1, Q2, Q3, Q4]);
    controller.custom_R = R;
    controller.custom_obs_factor = obs_factor;
    controller.custom_k_servo = k_servo;
    controller.custom_nl_scale = nl_scale;

    % Run the simulation.
    score = run_matlab_ball_and_beam(controller);
    fprintf('Parameters: Q=[%.2f, %.2f, %.2f, %.2f], R=%.2f, obs_factor=%.2f, k_servo=%.2f, nl_scale=%.2f, Score=%.4f\n',...
            Q1, Q2, Q3, Q4, R, obs_factor, k_servo, nl_scale, score);
end
