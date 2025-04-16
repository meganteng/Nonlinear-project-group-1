function [K, S, P] = manual_lqr(A, B, Q, R, N)
%manual_lqr Calculates LQR gain, Riccati solution, and poles for CT systems.

    % Input argument check
    if nargin < 4
        error('manual_lqr:NotEnoughInputs', 'Requires at least A, B, Q, and R matrices.');
    end

    if nargin < 5 || isempty(N)
        % Default N to zero if not provided or empty
        [n, m] = size(B);
        N = zeros(n, m);
    else
        [n_N, m_N] = size(N);
        [n, m] = size(B);
        [n_A, ~] = size(A);
        [n_Q, ~] = size(Q);
        [m_R, ~] = size(R);
        if n_N ~= n || m_N ~= m
             error('manual_lqr:IncorrectDimN', 'Matrix N must have dimensions n x m (%d x %d).', n, m);
        end
         if n_A ~= n || n_Q ~= n || m_R ~= m
             warning('manual_lqr:PotentialDimMismatch', 'Check dimensions of A, B, Q, R, N.');
         end
    end

    % Check properties (Basic checks, more robust checks like stabilizability/
    % observability are omitted for simplicity but are important in practice)
    if any(eig(R) <= 0)
        error('manual_lqr:RNotPositiveDefinite', 'Matrix R must be positive definite.');
    end
    % Check Q - N*inv(R)*N' >= 0 (semidefinite)
    Q_check = Q - N * (R \ N'); % Use backslash for efficiency and stability
    if any(eig(Q_check) < -sqrt(eps)) % Check with tolerance
         warning('manual_lqr:QNRCondition', 'Matrix [Q N''; N R] might not be positive semidefinite.');
    end

    % Solve the Continuous-time Algebraic Riccati Equation (ARE)
    % A'*S + S*A - (S*B + N)*inv(R)*(B'*S + N') + Q = 0
    try
        % Use MATLAB's built-in solver `care` for the ARE
        [S, ~, ~] = care(A, B, Q, R, N);
    catch ME
        error('manual_lqr:CAREError', 'Could not solve the Algebraic Riccati Equation using CARE. Error: %s', ME.message);
    end

    % Calculate the optimal gain K
    % K = inv(R) * (B'*S + N')
    K = R \ (B'*S + N'); % Use backslash operator (more stable than inv)

    % Calculate the closed-loop poles
    Acl = A - B*K;
    P = eig(Acl);

end
