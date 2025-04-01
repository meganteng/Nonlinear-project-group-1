% FILE: tune_fminsearch_LQR_full_feedback.m

% Define an initial guess with 5 elements: [Q1, Q2, R, k_servo, nl_scale]
p0 = [100, 0.3, 0.2, 10, 1];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective, p0, options);

fprintf('Optimal Parameters: Q=[%.2f, %.2f, 0.00, 0.00], R=%.2f, k_servo=%.2f, nl_scale=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4), opt_params(5));
fprintf('Best Score: %.4f\n', best_score);

// Parameters: Q=[158.10, 0.39, 0.00, 0.00], R=0.03, k_servo=7.24, nl_scale=1.30, Score=2.9715
//    107          209          2.97147         reflect
// Average Tracking Error: 0.0013 
// Average Energy Consumption: 0.1380 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.28 
// Energy Cost: 0.69 
// Safety Cost: 0.00 
// Total Score: 2.97 
