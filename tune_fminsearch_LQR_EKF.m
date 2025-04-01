% FILE: tune_fminsearch_LQR_EKF.m

% Define an initial guess for [Kp, Ki, Kd, obs_factor]
p0 = [300, 0.3, 0.4, 2];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective, p0, options);

fprintf('Optimal Parameters: Q11=%.2f, Q22=%.2f, R=%.2f, obs_factor=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.4f\n', best_score);


// Parameters: Q11=863.17, Q22=0.07, R=0.16, obs_factor=1.68, Score=2.4840
//     40           74          2.48404         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1357 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.81 
// Energy Cost: 0.68 
// Safety Cost: 0.00 
// Total Score: 2.48 
// Parameters: Q11=854.87, Q22=0.07, R=0.16, obs_factor=1.69, Score=2.4839


// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1381 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.76 
// Energy Cost: 0.69 
// Safety Cost: 0.00 
// Total Score: 2.45 
// Parameters: Q11=978.42, Q22=-0.01, R=0.17, obs_factor=1.59, Score=2.4520
// Warning: The [Q N;N' R] matrix should be positive semi-definite. Type "help lqr" for more information. 
// > In ss/lqr (line 57)
// In lqr (line 40)
// In studentControllerInterface/setupImpl (line 67)
// In run_matlab_ball_and_beam (line 33)
// In tune_objective (line 25)
// In fminsearch (line 343)
// In tune_fminsearch (line 10) 