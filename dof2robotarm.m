% animate_2dof_arm_with_snapshots.m
%
% This script animates a 2-degree-of-freedom (2-DOF) planar robotic arm
% following a random end-effector trajectory using Inverse Kinematics (IK).
% It visualizes the arm's movement and traces the end-effector's path.

clear; % Clears all variables from the workspace
clc;   % Clears the command window

%% --- 1. Define Robotic Arm Parameters ---
L1 = 1.0; % Length of Link 1 (in meters)
L2 = 0.8; % Length of Link 2 (in meters)

%% --- 2. Define End-Effector Path (Random Points) ---


num_path_points = 50; % Total number of random target points to generate for the path.
% Arm's theoretical max reach = L1 + L2 = 1.8m
% Arm's theoretical min reach = abs(L1 - L2) = 0.2m (from origin)

random_path_center_x = 0.5; % X-coordinate for the center of the random generation area
random_path_center_y = 0.8; % Y-coordinate for the center of the random generation area
random_max_deviation = 0.8; % Max deviation (radius) from the center for each point.

% Generate random X and Y coordinates for the target series.
% 'rand(1, num_path_points)' generates a 1x'num_path_points' array of random numbers between 0 and 1.
% '(2 * rand(...) - 1)' scales these numbers to be between -1 and 1.
% Multiplying by 'random_max_deviation' and adding the 'random_path_center'
% scales and shifts the random numbers to fit the desired square region.
x_target_series = random_path_center_x + (2 * rand(1, num_path_points) - 1) * random_max_deviation;
y_target_series = random_path_center_y + (2 * rand(1, num_path_points) - 1) * random_max_deviation;

%% --- 3. Setup Plot for Animation ---

figure('Renderer', 'painters', 'Position', [100 100 800 800]); 
hold on;   % Allows multiple plot elements to be added and updated on the same axes.
grid on;   % Adds a grid to the plot for easier visualization of coordinates.
axis equal; % Ensures equal scaling of X and Y axes, preventing distortion of the arm's shape.

% Set plot limits based on the arm's maximum theoretical reach to keep the arm centered and visible.
max_reach = L1 + L2;
plot_lim = max_reach * 1.1; 
xlim([-plot_lim, plot_lim]);
ylim([-plot_lim, plot_lim]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('2-DOF Robotic Arm: Random Path Following (IK)');
% Plot the fixed base joint at the origin (red circle).
plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); 


% Using NaN (Not a Number) as initial data so nothing is drawn until the first update.
h_link1 = plot(NaN, NaN, 'b-', 'LineWidth', 5); % Blue line for Link 1.
h_joint1 = plot(NaN, NaN, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); 
h_link2 = plot(NaN, NaN, 'g-', 'LineWidth', 5); 
h_ee     = plot(NaN, NaN, 'm*', 'MarkerSize', 12); 
h_target = plot(NaN, NaN, 'cx', 'MarkerSize', 15, 'LineWidth', 2); 

% Using 'r:' for a red dotted line for the trace.
h_path_trace = plot(NaN, NaN, 'r:', 'LineWidth', 1);

legend_handles = [h_link1, h_link2, h_ee, h_target, h_path_trace];
legend_names = {'Link 1', 'Link 2', 'End-Effector', 'Current Target', 'EE Trajectory'};
legend(legend_handles, legend_names, 'Location', 'best');

% Pre-allocation improves performance, especially in loops.
x_trace = zeros(1, num_path_points);
y_trace = zeros(1, num_path_points);

%% --- 4. Animation Loop for Path Following ---


print_every_N_frames = 5; % Controls how often information is printed to the Command Window.
                          % (e.g., 5 means print every 5th frame).

for i = 1:num_path_points
    xt = x_target_series(i); % Get the current target X-coordinate for this frame.
    yt = y_target_series(i); % Get the current target Y-coordinate for this frame.

    % --- Inverse Kinematics Calculation ---
    % Calls the 'solve_ik_2dof' function to find joint angles for the current target.
    % Ensure 'solve_ik_2dof.m' is saved as a separate file in the same directory.
    [theta1_rad_sol, theta2_rad_sol, reachable] = solve_ik_2dof(L1, L2, xt, yt);
    
    % Check if the current target point is reachable by the arm.
    if ~reachable
        fprintf('Frame %d: Target (%.2f, %.2f) unreachable! Skipping this point.\n', i, xt, yt);
        % If unreachable, store NaN for this point in the trace. This breaks the line
        % in the plot, preventing unwanted lines connecting unreachable gaps.
        x_trace(i) = NaN; 
        y_trace(i) = NaN;
        
        % Optionally, hide the arm when target is unreachable for visual clarity.
        set(h_link1, 'XData', [0, NaN], 'YData', [0, NaN]);
        set(h_joint1, 'XData', NaN, 'YData', NaN);
        set(h_link2, 'XData', [NaN, NaN], 'YData', [NaN, NaN]);
        set(h_ee, 'XData', NaN, 'YData', NaN);
        
        % Still update the target marker and the trace, and force redraw to keep animation smooth.
        set(h_target, 'XData', xt, 'YData', yt); 
        set(h_path_trace, 'XData', x_trace(1:i), 'YData', y_trace(1:i));
        drawnow limitrate;
        pause(0.05);
        continue; % Move to the next iteration of the loop.
    end

    % --- Choose one IK solution ---
    
    % from the previous configuration to ensure smoother physical motion.
    t1 = theta1_rad_sol(1); % Joint 1 angle from the chosen IK solution.
    t2 = theta2_rad_sol(1); % Joint 2 angle from the chosen IK solution.

   
    % using the IK-derived joint angles (t1, t2).
    P1_x = L1 * cos(t1);
    P1_y = L1 * sin(t1);
    xEE = P1_x + L2 * cos(t1 + t2);
    yEE = P1_y + L2 * sin(t1 + t2);

    % Store the current End-Effector position in the trace arrays.
    x_trace(i) = xEE;
    y_trace(i) = yEE;

    % Update the X and Y data properties of the plot handles to redraw the arm in its new position.
    set(h_link1, 'XData', [0, P1_x], 'YData', [0, P1_y]);
    set(h_joint1, 'XData', P1_x, 'YData', P1_y);
    set(h_link2, 'XData', [P1_x, xEE], 'YData', [P1_y, yEE]);
    set(h_ee, 'XData', xEE, 'YData', yEE);
    set(h_target, 'XData', xt, 'YData', yt); 
    
    % Update the continuous End-Effector trajectory trace on the plot.
    set(h_path_trace, 'XData', x_trace(1:i), 'YData', y_trace(1:i));

    
    if mod(i, print_every_N_frames) == 0
        fprintf('Frame %d: Target (%.2f, %.2f) -> Joint Angles (T1=%.1f°, T2=%.1f°)\n', ...
                i, xt, yt, ...
                rad2deg(t1), rad2deg(t2));
    end

    drawnow limitrate; 
    pause(0.1);       
end

hold off; % 