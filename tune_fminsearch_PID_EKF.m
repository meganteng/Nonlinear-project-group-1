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

// Average Tracking Error: 0.0009 
// Average Energy Consumption: 0.1720 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.55 
// Energy Cost: 0.86 
// Safety Cost: 0.00 
// Total Score: 2.41 
// Parameters: Kp=20.60, Ki=0.04, Kd=40.09, obs_factor=0.53, Score=2.4119

// Average Tracking Error: 0.0012 
// Average Energy Consumption: 0.1152 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.19 
// Energy Cost: 0.58 
// Safety Cost: 0.00 
// Total Score: 2.76 
// Parameters: Kp=6.06, Ki=-0.01, Kd=13.02, obs_factor=0.84, Score=2.7641

// Average Tracking Error: 0.0014 
// Average Energy Consumption: 0.1077 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.59 
// Energy Cost: 0.54 
// Safety Cost: 0.00 
// Total Score: 3.13 
// Parameters: Kp=4.35, Ki=0.15, Kd=11.53, obs_factor=1.04, Score=3.1256

// Average Tracking Error: 0.0014 
// Average Energy Consumption: 0.1091 
// Safety Contraint Violation: 1 
// Tracking Cost: 2.50 
// Energy Cost: 0.55 
// Safety Cost: 10.00 
// Total Score: 13.04 
// Parameters: Kp=4.61, Ki=0.11, Kd=11.88, obs_factor=0.99, Score=13.0413