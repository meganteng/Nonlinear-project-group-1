function K_mx = preset_values()

% Define system parameters
g = 9.81;   % Gravity (m/s^2)
tau = 0.025; % Motor time constant (s)
K_motor = 1.5; % Motor gain (rad/sV)
rg = 0.0254; % Servo arm length (m)
L = 0.4255; % Beam length (m)

% Define updated A, B matrices with tau included
A = [0 1 0 0; 
     0 0 5*g*rg/(7*L) 0; 
     0 0 0 1; 
     0 0 0 -1/tau];

B = [0; 0; 0; K_motor/tau];

% Define LQR weight matrices
Q = diag([500, 0.3, 0, 0]); % Adjusted Q for smoother control
R = 5;  % Increased control effort penalty

% Compute LQR gain
K_mx = lqr(A, B, Q, R);

end