% FILE: tune_fminsearch_PID.m

% Define an initial guess for parameters: [Kp, Ki, Kd]
p0 = [3, 1, 5];

% Set optimization options.
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the PID parameters (minimize score).
[opt_params, best_score] = fminsearch(@tune_objective_PID, p0, options);

fprintf('Optimal Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', ...
    opt_params(1), opt_params(2), opt_params(3));
fprintf('Best Score: %.4f\n', best_score);
