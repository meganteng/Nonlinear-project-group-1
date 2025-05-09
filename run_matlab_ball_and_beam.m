% Observer states plotting available. Please read the comments. - Megan
close all
clear
[Ad, Bd, Cd, K_mx, L_mx] = preset_values();

%% Change controller names here to use the specific student controller.
% LQR_FL_Luenberger: Continuous time LQR + Feedback Linearization + Luenburger Observer.
% LQR_FL_Luenberger_DT: Discrete time LQR + Feedback Linearization + Luenburger Observer.
% LQR_full_feedback: Continuous time LQR + Feedback Linearization + Discrete time linearization.
% LQR_Local: Continuous time LQR + Local linearization. This controller will lead to ewarly termination.
% PID: Continuous time PID.
controller_name = "LQR_FL_Luenberger";

%% General Settings.
% Initial state.
x0 = [-0.19; 0.00; 0; 0];
t0 = 0;
% Simulation time.
T = 90;
% Sampling time of the controller
dt = 0.01;
% ode function to use.
ode_func = @ode45;
% print log for each timestep if true.
verbose = false;
% plot animation if true.
plot_animation = true;
% save animation to video if true.
save_video = false;
    
if controller_name == "LQR_FL_Luenberger_DT"
    controller_handle = studentControllerInterface_LQR_FL_Luenberger_DT();
elseif controller_name == "LQR_full_feedback"
    controller_handle = studentControllerInterface_LQR_full_feedback();
elseif controller_name == "PID"
    controller_handle = studentControllerInterface_PID();
elseif controller_name == "LQR_Local"
    controller_handle = studentControllerInterface_LQR_Local();
else
    controller_handle = studentControllerInterface();
end
setup(controller_handle);
u_saturation = 10;

% Initialize traces.
xs = x0;
% Uncomment the following line if you wish to plot out the observer states.
% xhat = x0;
ts = t0;
us = [];
theta_ds = [];
[p_ball_ref, v_ball_ref] = get_ref_traj(t0);
ref_ps = p_ball_ref;
ref_vs = v_ball_ref;

% Initialize state & time.
x = x0;
t = t0;
end_simulation = false;
%% Run simulation.
% _t indicates variables for the current loop.
tstart = tic;
while ~end_simulation
    %% Determine control input.
    tstart = tic; % DEBUG    
    [u, theta_d] = controller_handle.stepController(t, x(1), x(3), Ad, Bd, Cd, K_mx, L_mx);
    % Uncomment the following line if you wish to plot out the observer
    % states.
    % xhat_current = controller_handle.getObserverStates();
    u = min(u, u_saturation);
    u = max(u, -u_saturation);
    if verbose
        print_log(t, x, u);    
    end
    tend = toc(tstart);    
    us = [us, u];          
    theta_ds = [theta_ds, theta_d];
    %% Run simulation for one time step.
    t_end_t = min(t + dt, t0+T);
    ode_opt = odeset('Events', @event_ball_out_of_range);
    [ts_t, xs_t, t_event] = ode_func( ...
        @(t, x) ball_and_beam_dynamics(t, x, u), ...
        [t, t_end_t], x, ode_opt);
    end_simulation = abs(ts_t(end) - (t0 + T))<1e-10 || ~isempty(t_event);
    end_with_event = ~isempty(t_event); 
    t = ts_t(end);
    x = xs_t(end, :)';
    %% Record traces.
    xs = [xs, x];
    % Uncomment the following line if you wish to plot out the observer states.
    % xhat = [xhat, xhat_current];
    ts = [ts, t];
    [p_ball_ref, v_ball_ref] = get_ref_traj(t);
    ref_ps = [ref_ps, p_ball_ref];
    ref_vs = [ref_vs, v_ball_ref];    
end % end of the main while loop
%% Add control input for the final timestep.
[u, theta_d] = controller_handle.stepController(t, x(1), x(3), Ad, Bd, Cd, K_mx, L_mx);
u = min(u, u_saturation);
u = max(u, -u_saturation);
us = [us, u];
theta_ds = [theta_ds, theta_d];
if verbose
    print_log(t, x, u);    
end
ps = xs(1, :);
thetas = xs(3, :);

% Evaluate the score of the controller.
score = get_controller_score(ts, ps, thetas, ref_ps, us);

%% Plots
% Plot states.
plot_states(ts, xs, ref_ps, ref_vs, theta_ds);

% Uncomment the following line if you wish to plot out the observer states.
% plot_observer_states(ts, xs, xhat, ref_ps, ref_vs, theta_ds);

% Plot output errors.
plot_tracking_errors(ts, ps, ref_ps);        
% Plot control input history.
plot_controls(ts, us);

if plot_animation
    animate_ball_and_beam(ts, ps, thetas, ref_ps, save_video);
end

function print_log(t, x, u)
        fprintf('t: %.3f, \t x: ', t);
        fprintf('%.2g, ', x);
        fprintf('\t u: ');
        fprintf('%.2g, ', u);
        % Add custom log here.
        fprintf('\n');
end