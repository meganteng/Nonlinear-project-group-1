% FILE: tune_fminsearch_LQR_EKF_fixed_Q22.m

% Define an initial guess for [Q11, R, obs_factor]
p0 = [300, 0.4, 2];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective_LQR_EKF_fixed_Q22, p0, options);

fprintf('Optimal Parameters: Q11=%.2f, Q22=%.2f, R=%.2f, obs_factor=%.2f\n',...
    opt_params(1), 0, opt_params(2), opt_params(3));
fprintf('Best Score: %.4f\n', best_score);
