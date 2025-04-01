% FILE: tune_fminsearch_PID.m

% Define an initial guess for parameters: [Kp, Ki, Kd, k_servo]
p0 = [3, 1, 5, 5];

% Set optimization options.
options = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2);

% Optimize the PID parameters (minimize score).
[opt_params, best_score] = fminsearch(@tune_objective, p0, options);

fprintf('Optimal Parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, k_servo=%.2f\n', ...
    opt_params(1), opt_params(2), opt_params(3), opt_params(4));
fprintf('Best Score: %.4f\n', best_score);


// Average Tracking Error: 0.0009 
// Average Energy Consumption: 0.1292 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.62 
// Energy Cost: 0.65 
// Safety Cost: 0.00 
// Total Score: 2.27 
// Parameters: Kp=17.45, Ki=-0.07, Kd=15.26, k_servo=6.63, Score=2.2676

// Average Tracking Error: 0.0016 
// Average Energy Consumption: 0.0816 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.84 
// Energy Cost: 0.41 
// Safety Cost: 0.00 
// Total Score: 3.24 
// Parameters: Kp=6.44, Ki=0.10, Kd=11.15, k_servo=2.17, Score=3.2434
// Parameters: Kp=7.13, Ki=0.10, Kd=11.82, k_servo=1.11, Score=Inf

// Average Tracking Error: 0.0018 
// Average Energy Consumption: 0.0822 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.31 
// Energy Cost: 0.41 
// Safety Cost: 0.00 
// Total Score: 3.72 
// Parameters: Kp=4.12, Ki=0.05, Kd=8.88, k_servo=6.14, Score=3.7198
//     45           83          3.65869         contract inside
// Average Tracking Error: 0.0018 
// Average Energy Consumption: 0.0823 
// Safety Contraint Violation: 1 
// Tracking Cost: 3.26 
// Energy Cost: 0.41 
// Safety Cost: 10.00 
// Total Score: 13.67 
// Parameters: Kp=4.15, Ki=0.03, Kd=8.95, k_servo=6.16, Score=13.6732