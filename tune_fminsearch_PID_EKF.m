% FILE: tune_fminsearch_PID_EKF.m

% Define an initial guess for [Kp, Ki, Kd, obs_factor]
p0 = [3, 1, 5, 1];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective, p0, options);

fprintf('Optimal Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, obs_factor=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.4f\n', best_score);
