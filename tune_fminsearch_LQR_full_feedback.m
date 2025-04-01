% FILE: tune_fminsearch_LQR_full_feedback.m

% Define an initial guess with 4 elements: [Q1, Q2, R, nl_scale]
p0 = [100, 0.3, 0.2, 1];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective_LQR_full_feedback, p0, options);

fprintf('Optimal Parameters: Q=[%.2f, %.2f, 0.00, 0.00], R=%.2f, nl_scale=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.4f\n', best_score);
