% FILE: tune_fminsearch_LQR_FL_Luenberger.m

% Define an initial guess (try the candidate above)
p0 = [300, 0.3, 0.4, 2];

% Set optimization options (display iterations, tolerance, etc.)
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the objective function (minimizing score)
[opt_params, best_score] = fminsearch(@tune_objective_LQR_FL_Luenberger, p0, options);

fprintf('Optimal Parameters: Q11=%.8f, Q22=%.8f, R=%.8f, obs_factor=%.8f\n',...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.8f\n', best_score);



// Parameters: Q11=50.188321, Q22=0.499481, R=0.099787, obs_factor=2.099336, Score=16.558934
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5367 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.92 
// Energy Cost: 2.68 
// Safety Cost: 0.00 
// Total Score: 6.60 
// Parameters: Q11=50.177160, Q22=0.499243, R=0.099808, obs_factor=2.099670, Score=6.602966
//     28           65          6.52533         contract inside
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5266 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.91 
// Energy Cost: 2.63 
// Safety Cost: 0.00 
// Total Score: 6.54 
// Parameters: Q11=50.247779, Q22=0.498506, R=0.099795, obs_factor=2.099675, Score=6.542197
//     29           66          6.52533         reflect
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5330 
// Safety Contraint Violation: 1 
// Tracking Cost: 3.91 
// Energy Cost: 2.66 
// Safety Cost: 10.00 
// Total Score: 16.57 
// Parameters: Q11=50.240970, Q22=0.498735, R=0.099788, obs_factor=2.099526, Score=16.572747
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5181 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.92 
// Energy Cost: 2.59 
// Safety Cost: 0.00 
// Total Score: 6.51 
// Parameters: Q11=50.193112, Q22=0.499116, R=0.099803, obs_factor=2.099634, Score=6.505580
//     30           68          6.50558         contract inside
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5242 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.92 
// Energy Cost: 2.62 
// Safety Cost: 0.00 
// Total Score: 6.54 
// Parameters: Q11=50.143017, Q22=0.499776, R=0.099805, obs_factor=2.099500, Score=6.539190
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5179 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.92 
// Energy Cost: 2.59 
// Safety Cost: 0.00 
// Total Score: 6.51 
// Parameters: Q11=50.169207, Q22=0.499459, R=0.099803, obs_factor=2.099544, Score=6.505281
//     31           70          6.50528         contract outside
// Average Tracking Error: 0.0022 
// Average Energy Consumption: 0.5353 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.92 
// Energy Cost: 2.68 
// Safety Cost: 0.00 
// Total Score: 6.59 
// Parameters: Q11=50.222896, Q22=0.498968, R=0.099758, obs_factor=2.099744, Score=6.594676
