% FILE: tune_fminsearch_LFL_Luen_fixObs.m

% Define an initial guess (not strictly needed by ga, but good practice)
% Use the best result from GA as the initial guess for fminsearch
p0 = [298.29808108, 6.91249284, 0.40564243];

% Define parameter bounds [Q11, Q22, R] (Note: fminsearch does not directly use bounds)
lb = [1,   0.01, 0.01]; % Lower bounds
ub = [1000, 10,  10]; % Upper bounds

% Number of variables
nvars = length(p0); % Should be 3

% Set optimization options for GA (commented out)
% options = optimoptions('ga', ...
%                        'Display','iter', ...
%                        'FunctionTolerance', 1e-2);

% Set optimization options for fminsearch
options = optimset('Display','iter', ...        % Display iteration info
                   'TolFun', 1e-4, ...       % Function tolerance
                   'TolX', 1e-4, ...         % Parameter tolerance
                   'MaxIter', 500);         % Maximum iterations

% Optimize using patternsearch (commented out)
% [opt_params, best_score] = patternsearch(@tune_objective_LQR_FL_Luenberger, p0, [], [], [], [], lb, ub, [], options);

% Optimize using ga (genetic algorithm) - commented out
% [opt_params, best_score] = ga(@tune_objective_LFL_Luen_fixObs, nvars, [], [], [], [], lb, ub, [], options);

% Call fminsearch with the updated initial guess
[opt_params, best_score] = fminsearch(@tune_objective_LFL_Luen_fixObs, p0, options);

% Print results
fprintf('Optimal Parameters: Q11=%.8f, Q22=%.8f, R=%.8f\n',...
    opt_params(1), opt_params(2), opt_params(3));
fprintf('Best Score: %.8f\n', best_score);


// Average Tracking Error: 0.0021 
// Average Energy Consumption: 0.1552 
// Safety Contraint Violation: 0 
// Tracking Cost: 3.75 
// Energy Cost: 0.78 
// Safety Cost: 0.00 
// Total Score: 4.53 
// Parameters: Q11=298.00610111, Q22=6.87701955, R=0.40565694, Score=4.52791990
//     76          159          4.52792         contract inside
 
// Optimization terminated:
//  the current x satisfies the termination criteria using OPTIONS.TolX of 1.000000e-04 
//  and F(X) satisfies the convergence criteria using OPTIONS.TolFun of 1.000000e-04 

// Optimal Parameters: Q11=298.00600625, Q22=6.87702042, R=0.40565645
// Best Score: 4.52791989
// >> 