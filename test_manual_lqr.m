%% Test script for manual_lqr vs built-in lqr

clear; clc; close all;

tol = 1e-8; % Tolerance for comparison

%% Test Case 1: Inverted Pendulum
fprintf('--- Test Case 1: Inverted Pendulum ---\n');

% System matrices (Example from lqr documentation - corrected B)
% State: [x; x_dot; theta; theta_dot]
A_pend = [ 0   1   0       0; 
           0  -0.1 0.5     0; % Values might differ based on specific model
           0   0   0       1; 
           0  -3  30       0]; 
B_pend = [0; 2; 0; 5]; % Input u affects x_ddot and theta_ddot

% Weighting matrices (Bryson's rule example)
Q_pend = diag([1, 0, 1, 0]); % Penalize x and theta
R_pend = 1;
N_pend = zeros(4, 1); % No cross-term

% Using built-in lqr
fprintf('Running built-in lqr...\n');
[K_builtin_p, S_builtin_p, P_builtin_p_unsorted] = lqr(A_pend, B_pend, Q_pend, R_pend, N_pend);
P_builtin_p = sort(P_builtin_p_unsorted); % Sort poles for consistent comparison

% Using manual_lqr
fprintf('Running manual_lqr...\n');
try
    [K_manual_p, S_manual_p, P_manual_p_unsorted] = manual_lqr(A_pend, B_pend, Q_pend, R_pend, N_pend);
    P_manual_p = sort(P_manual_p_unsorted); % Sort poles

    % Compare results
    diff_K_p = norm(K_builtin_p - K_manual_p);
    diff_S_p = norm(S_builtin_p - S_manual_p);
    diff_P_p = norm(P_builtin_p - P_manual_p);

    fprintf('Difference in K: %e\n', diff_K_p);
    fprintf('Difference in S: %e\n', diff_S_p);
    fprintf('Difference in P: %e\n', diff_P_p);

    if diff_K_p < tol && diff_S_p < tol && diff_P_p < tol
        fprintf('PASS: Results match for Inverted Pendulum.\n\n');
    else
        fprintf('FAIL: Results DO NOT match for Inverted Pendulum.\n\n');
    end
catch ME
    fprintf('FAIL: Error running manual_lqr for Inverted Pendulum: %s\n\n', ME.message);
end

%% Test Case 2: Aircraft Pitch
fprintf('--- Test Case 2: Aircraft Pitch ---\n');

% System matrices (Example from lqr documentation)
% State: [alpha; q; theta]
A_ac = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B_ac = [0.232; 0.0203; 0]; % Input delta_e
C_ac = [0 0 1]; % Output theta
D_ac = 0;

% Weighting matrices
Q_ac = 25 * (C_ac' * C_ac); % Using Q2 from example (penalize theta)
R_ac = 1;
N_ac = zeros(3, 1); % No cross-term

% Using built-in lqr
fprintf('Running built-in lqr...\n');
[K_builtin_ac, S_builtin_ac, P_builtin_ac_unsorted] = lqr(A_ac, B_ac, Q_ac, R_ac, N_ac);
P_builtin_ac = sort(P_builtin_ac_unsorted); % Sort poles

% Using manual_lqr
fprintf('Running manual_lqr...\n');
try
    [K_manual_ac, S_manual_ac, P_manual_ac_unsorted] = manual_lqr(A_ac, B_ac, Q_ac, R_ac, N_ac);
    P_manual_ac = sort(P_manual_ac_unsorted); % Sort poles

    % Compare results
    diff_K_ac = norm(K_builtin_ac - K_manual_ac);
    diff_S_ac = norm(S_builtin_ac - S_manual_ac);
    diff_P_ac = norm(P_builtin_ac - P_manual_ac);

    fprintf('Difference in K: %e\n', diff_K_ac);
    fprintf('Difference in S: %e\n', diff_S_ac);
    fprintf('Difference in P: %e\n', diff_P_ac);

    if diff_K_ac < tol && diff_S_ac < tol && diff_P_ac < tol
        fprintf('PASS: Results match for Aircraft Pitch.\n\n');
    else
        fprintf('FAIL: Results DO NOT match for Aircraft Pitch.\n\n');
    end
catch ME
    fprintf('FAIL: Error running manual_lqr for Aircraft Pitch: %s\n\n', ME.message);
end

fprintf('--- Test Complete ---\n');