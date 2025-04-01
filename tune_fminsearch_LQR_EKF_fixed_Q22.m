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


// Parameters: Q11=2483.82, Q22=0.00, R=0.27, obs_factor=2.55, Score=Inf

// Average Tracking Error: 0.0009 
// Average Energy Consumption: 0.1446 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.62 
// Energy Cost: 0.72 
// Safety Cost: 0.00 
// Total Score: 2.34 
// Parameters: Q11=2481.92, Q22=0.00, R=0.27, obs_factor=2.55, Score=2.3408

// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1307 
// Safety Contraint Violation: 1 
// Tracking Cost: 2.07 
// Energy Cost: 0.65 
// Safety Cost: 10.00 
// Total Score: 12.72 
// Parameters: Q11=412.37, Q22=0.00, R=0.12, obs_factor=2.20, Score=12.7220

// Average Tracking Error: 0.0012 
// Average Energy Consumption: 0.1246 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.14 
// Energy Cost: 0.62 
// Safety Cost: 0.00 
// Total Score: 2.77 
// Parameters: Q11=409.56, Q22=0.00, R=0.14, obs_factor=2.24, Score=2.7670




// Perhaps here is where it is somewhat tune_studentControllerInterface_LQR_FL_Luenberger
// Parameters: Q11=480.20, Q22=0.00, R=0.14, obs_factor=2.24, Score=2.6763
//     36           68          2.67628         expand
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1291 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.03 
// Energy Cost: 0.65 
// Safety Cost: 0.00 
// Total Score: 2.67 
// Parameters: Q11=481.15, Q22=0.00, R=0.14, obs_factor=2.25, Score=2.6711
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1295 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.00 
// Energy Cost: 0.65 
// Safety Cost: 0.00 
// Total Score: 2.65 
// Parameters: Q11=506.57, Q22=0.00, R=0.14, obs_factor=2.25, Score=2.6505
//     37           70          2.65047         expand
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1281 
// Safety Contraint Violation: 0 
// Tracking Cost: 2.01 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.65 
// Parameters: Q11=525.29, Q22=0.00, R=0.14, obs_factor=2.25, Score=2.6460
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1275 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.99 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.62 
// Parameters: Q11=570.77, Q22=0.00, R=0.15, obs_factor=2.26, Score=2.6225
//     38           72          2.62248         expand
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1283 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.97 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.61 
// Parameters: Q11=585.72, Q22=0.00, R=0.15, obs_factor=2.26, Score=2.6088
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1282 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.94 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.58 
// Parameters: Q11=652.25, Q22=0.00, R=0.16, obs_factor=2.26, Score=2.5778
//     39           74          2.57779         expand
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1282 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.93 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.57 
// Parameters: Q11=672.86, Q22=0.00, R=0.16, obs_factor=2.27, Score=2.5691
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1279 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.90 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.54 
// Parameters: Q11=769.18, Q22=0.00, R=0.17, obs_factor=2.28, Score=2.5367
//     40           76          2.53669         expand
// Average Tracking Error: 0.0011 
// Average Energy Consumption: 0.1266 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.90 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.53 
// Parameters: Q11=821.56, Q22=0.00, R=0.18, obs_factor=2.28, Score=2.5312
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1255 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.88 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.51 
// Parameters: Q11=979.05, Q22=0.00, R=0.21, obs_factor=2.30, Score=2.5065
//     41           78          2.50648         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1265 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.86 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.49 
// Parameters: Q11=1029.56, Q22=0.00, R=0.21, obs_factor=2.31, Score=2.4904
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1261 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.84 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.47 
// Parameters: Q11=1258.95, Q22=0.00, R=0.24, obs_factor=2.33, Score=2.4703
//     42           80          2.47031         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1253 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.85 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.47 
// Parameters: Q11=1352.54, Q22=0.00, R=0.25, obs_factor=2.35, Score=2.4722
//     43           81          2.47031         reflect
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1245 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.85 
// Energy Cost: 0.62 
// Safety Cost: 0.00 
// Total Score: 2.48 
// Parameters: Q11=1624.52, Q22=0.00, R=0.29, obs_factor=2.37, Score=2.4752
//     44           82          2.47031         reflect
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1251 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.85 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.47 
// Parameters: Q11=1844.95, Q22=0.00, R=0.32, obs_factor=2.40, Score=2.4707
//     45           83          2.47031         reflect
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1266 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.83 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.46 
// Parameters: Q11=1346.45, Q22=0.00, R=0.25, obs_factor=2.35, Score=2.4604
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1281 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.82 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.46 
// Parameters: Q11=1207.42, Q22=0.00, R=0.22, obs_factor=2.34, Score=2.4574
//     46           85          2.45744         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1270 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.82 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.45 
// Parameters: Q11=1521.67, Q22=0.00, R=0.26, obs_factor=2.36, Score=2.4505
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1278 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.80 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.44 
// Parameters: Q11=1606.24, Q22=0.00, R=0.27, obs_factor=2.37, Score=2.4420
//     47           87          2.44198         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1326 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.82 
// Energy Cost: 0.66 
// Safety Cost: 0.00 
// Total Score: 2.48 
// Parameters: Q11=870.12, Q22=0.00, R=0.17, obs_factor=2.29, Score=2.4828
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1260 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.83 
// Energy Cost: 0.63 
// Safety Cost: 0.00 
// Total Score: 2.46 
// Parameters: Q11=1601.24, Q22=0.00, R=0.28, obs_factor=2.37, Score=2.4590
//     48           89          2.44198         contract inside
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1281 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.80 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.44 
// Parameters: Q11=1684.31, Q22=0.00, R=0.28, obs_factor=2.39, Score=2.4381
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1289 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.79 
// Energy Cost: 0.64 
// Safety Cost: 0.00 
// Total Score: 2.43 
// Parameters: Q11=1896.99, Q22=0.00, R=0.30, obs_factor=2.42, Score=2.4321
//     49           91           2.4321         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1310 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.76 
// Energy Cost: 0.65 
// Safety Cost: 0.00 
// Total Score: 2.42 
// Parameters: Q11=1539.19, Q22=0.00, R=0.25, obs_factor=2.38, Score=2.4177
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1341 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.73 
// Energy Cost: 0.67 
// Safety Cost: 0.00 
// Total Score: 2.40 
// Parameters: Q11=1508.16, Q22=0.00, R=0.23, obs_factor=2.38, Score=2.3988
//     50           93           2.3988         expand
// Average Tracking Error: 0.0010 
// Average Energy Consumption: 0.1312 
// Safety Contraint Violation: 0 
// Tracking Cost: 1.76 
// Energy Cost: 0.66 
// Safety Cost: 0.00 
// Total Score: 2.42 
// Parameters: Q11=2133.52, Q22=0.00, R=0.31, obs_factor=2.45, Score=2.4162
