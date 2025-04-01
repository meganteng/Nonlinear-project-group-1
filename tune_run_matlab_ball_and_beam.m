% FILE: tune_run_matlab_ball_and_beam.m

function score = run_matlab_ball_and_beam(controller)
% run_matlab_ball_and_beam - Run the ball-and-beam simulation with a given controller.
%
% Input:
%   controller - instance of studentControllerInterface with custom parameters set
%
% Output:
%   score - computed performance score from get_controller_score

    % Observer states plotting available. Please read the comments. - Megan
    close all
    clearvars -except controller  % avoid clearing the controller instance

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
    plot_animation = false;
    % save animation to video if true.
    save_video = false;

    % The controller instance is provided as input
    setup(controller);
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
    while ~end_simulation
        %% Determine control input.
        [u, theta_d] = controller.stepController(t, x(1), x(3));
        u = min(u, u_saturation);
        u = max(u, -u_saturation);
        if verbose
            print_log(t, x, u);
        end
        us = [us, u];
        theta_ds = [theta_ds, theta_d];
        %% Run simulation for one time step.
        t_end_t = min(t + dt, t0+T);
        ode_opt = odeset('Events', @event_ball_out_of_range);
        [ts_t, xs_t, t_event] = ode_func( ...
            @(t, x) ball_and_beam_dynamics(t, x, u), ...
            [t, t_end_t], x, ode_opt);
        end_simulation = abs(ts_t(end) - (t0 + T)) < 1e-10 || ~isempty(t_event);
        t = ts_t(end);
        x = xs_t(end, :)';
        %% Record traces.
        xs = [xs, x];
        ts = [ts, t];
        [p_ball_ref, v_ball_ref] = get_ref_traj(t);
        ref_ps = [ref_ps, p_ball_ref];
        ref_vs = [ref_vs, v_ball_ref];
    end % end of the main while loop

    %% Add control input for the final timestep.
    [u, theta_d] = controller.stepController(t, x(1), x(3));
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
    plot_states(ts, xs, ref_ps, ref_vs, theta_ds);
    % Uncomment the following line if you wish to plot out the observer states.
    % plot_observer_states(ts, xs, xhat, ref_ps, ref_vs, theta_ds);
    plot_tracking_errors(ts, ps, ref_ps);
    plot_controls(ts, us);

    if plot_animation
        animate_ball_and_beam(ts, ps, thetas, ref_ps, save_video);
    end

    function print_log(t, x, u)
        fprintf('t: %.3f, \t x: ', t);
        fprintf('%.2g, ', x);
        fprintf('\t u: ');
        fprintf('%.2g, ', u);
        fprintf('\n');
    end

end
