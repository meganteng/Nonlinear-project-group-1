function [Ad, Bd, Cd, K_mx, L_mx] = preset_values()

 % Define system parameters
g = 9.81;   % Gravity (m/s^2)
tau = 0.025; % Motor time constant (s)
K_motor = 1.5; % Motor gain (rad/sV)
rg = 0.0254; % Servo arm length (m)
L_beam = 0.4255; % Beam length (m)

% Define system matrices (A, B, C)
Ad = [0 1 0 0;
    0 0 5*g*rg/(7*L_beam) 0;
    0 0 0 1;
    0 0 0 -1/tau];
Bd = [0; 0; 0; K_motor/tau];
Cd = [1 0 0 0;
     0 0 1 0];
% Q = diag([298, 6.87, 0, 0]); % Fill in your optimal Q matrix here
% R = 0.406;    % Fill in your optimal R value here
Q = diag([100, 0, 0, 0]);
R = 20;
dt = 0.01;  % Sampling time

% Discretize A, B using zero-order hold
% [Ad, Bd] = c2d(Ac, Bc, dt);

K_mx = lqr(Ad, Bd, Q, R);

% Observer design: place discrete poles (can multiply continuous poles' magnitudes)
manual_pole_radius = 0.3;
manual_poles = manual_pole_radius * [0.5, 0.5, 0.9, 0.9];
L_mx = place(Ad', Cd', manual_poles)';  % Note the transpose

end