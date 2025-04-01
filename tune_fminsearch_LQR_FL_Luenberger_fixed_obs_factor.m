% FILE: tune_fminsearch_LQR_FL_Luenberger_fixed_obs_factor.m

% Define an initial guess (not strictly needed by ga, but good practice)
p0 = [300, 0.3, 0.4];

% Define parameter bounds [Q11, Q22, R]
lb = [1,   0.01, 0.01]; % Lower bounds
ub = [1000, 10,  10]; % Upper bounds

% Number of variables
nvars = length(p0); % Should be 3

% Set optimization options for GA
options = optimoptions('ga', ...
                       'Display','iter', ...
                       'FunctionTolerance', 1e-2); % Use FunctionTolerance for ga
                       % PopulationSize could be increased for wider search, e.g., 'PopulationSize', 100
                       % MaxGenerations could be set if needed

% Optimize using patternsearch (commented out)
% [opt_params, best_score] = patternsearch(@tune_objective_LQR_FL_Luenberger, p0, [], [], [], [], lb, ub, [], options);

% Optimize using ga (genetic algorithm)
[opt_params, best_score] = ga(@tune_objective_LQR_FL_Luenberger, nvars, [], [], [], [], lb, ub, [], options);


fprintf('Optimal Parameters: Q11=%.8f, Q22=%.8f, R=%.8f\n',...
    opt_params(1), opt_params(2), opt_params(3));
fprintf('Best Score: %.8f\n', best_score);