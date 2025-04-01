% FILE: tune_fminsearch.m

p0 = [300, 0.3, 0, 0, 0.4, 2, 8, 1];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective, p0, options);

fprintf('Optimal Parameters: Q=[%.2f, %.2f, %.2f, %.2f], R=%.2f, obs_factor=%.2f, k_servo=%.2f, nl_scale=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4), opt_params(5), opt_params(6), opt_params(7), opt_params(8));
fprintf('Best Score: %.4f\n', best_score);


// Parameters: Q=[440.12, 0.14, 0.00, 0.00], R=0.23, obs_factor=1.94, k_servo=7.69, nl_scale=0.68, Score=3.1790
//    271          451          3.12662         contract inside
// Average Tracking Error: 0.0015 
// Average Energy Consumption: 0.1098 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.67 
// Energy Cost: 0.55 
// Safety Cost: 0.00 
// Total Score: 3.22 