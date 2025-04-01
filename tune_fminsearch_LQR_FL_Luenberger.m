% FILE: tune_fminsearch_LQR_FL_Luenberger.m

% Define an initial guess (try the candidate above)
p0 = [300, 0.3, 0.4, 2];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective_LQR_FL_Luenberger, p0, options);

fprintf('Optimal Parameters: Q11=%.2f, Q22=%.2f, R=%.2f, obs_factor=%.2f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.4f\n', best_score);




// Parameters: Q11=300.76, Q22=0.30, R=0.40, obs_factor=2.10, Score=6.6348
//     57          126          6.62948         contract inside
// Average Tracking Error: 0.0020 
// Average Energy Consumption: 0.6197 
// Safety Contraint Violation: 1 
// Tracking Cost: 3.53 
// Energy Cost: 3.10 
// Safety Cost: 10.00 
// Total Score: 16.63 
// Parameters: Q11=300.75, Q22=0.30, R=0.40, obs_factor=2.10, Score=16.6285
// Average Tracking Error: 0.0020 
// Average Energy Consumption: 0.6210 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.53 
// Energy Cost: 3.10 
// Safety Cost: 0.00 
// Total Score: 6.63 
// Parameters: Q11=300.76, Q22=0.30, R=0.40, obs_factor=2.10, Score=6.6350
//     58          128          6.62948         contract inside
// Average Tracking Error: 0.0020 
// Average Energy Consumption: 0.6198 
// Safety Contraint Violation: 1 
// Tracking Cost: 3.53 
// Energy Cost: 3.10 
// Safety Cost: 10.00 
// Total Score: 16.63 
// Parameters: Q11=300.75, Q22=0.30, R=0.40, obs_factor=2.10, Score=16.6289



// Optimization terminated:
//  the current x satisfies the termination criteria using OPTIONS.TolX of 1.000000e-02 
//  and F(X) satisfies the convergence criteria using OPTIONS.TolFun of 1.000000e-02 

// Optimal Parameters: Q11=300.74, Q22=0.30, R=0.40, obs_factor=2.10
// Best Score: 6.6295