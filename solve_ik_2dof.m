function [theta1_rad_sol, theta2_rad_sol, reachable] = solve_ik_2dof(L1, L2, x_target, y_target)
%SOLVE_IK_2DOF Calculates inverse kinematics for a 2-DOF planar arm.
%   [theta1_rad_sol, theta2_rad_sol, reachable] = solve_ik_2dof(L1, L2, x_target, y_target)
%   
%   Inputs:
%     L1       : Length of the first link
%     L2       : Length of the second link
%     x_target : Desired X-coordinate of the end-effector
%     y_target : Desired Y-coordinate of the end-effector
%
%   Outputs:
%     theta1_rad_sol : A 1x2 array of possible theta1 solutions in radians
%     theta2_rad_sol : A 1x2 array of possible theta2 solutions in radians
%     reachable      : Boolean, true if target is reachable, false otherwise

    % Initialize outputs
    theta1_rad_sol = [NaN, NaN];
    theta2_rad_sol = [NaN, NaN];
    reachable = false;

    % 1. Calculate distance from base to target (D)
    D_squared = x_target^2 + y_target^2;
    D = sqrt(D_squared);

    % 2. Check Reachability (Workspace limits)
    min_reach = abs(L1 - L2);
    max_reach = L1 + L2;

    % If the target is outside the arm's physical reach, it's unreachable
    if D > max_reach || D < min_reach
        % Optionally, you can uncomment the line below to see a message in the Command Window
        % disp('Target is unreachable!'); 
        return; % Exit function early if unreachable
    end

    % 3. Calculate theta2 using Law of Cosines
    % The formula is derived from the Law of Cosines applied to the triangle formed by
    % the base, joint 1, and the end-effector (sides L1, L2, and D).
    cos_theta2 = (D_squared - L1^2 - L2^2) / (2 * L1 * L2);
    
    % Handle potential numerical errors: acos requires input in [-1, 1]
    % Floating-point arithmetic can sometimes cause values to be slightly outside this range.
    if cos_theta2 > 1
        cos_theta2 = 1;
    elseif cos_theta2 < -1
        cos_theta2 = -1;
    end

    % Calculate the base angle for theta2. acos returns values in [0, pi]
    theta2_base = acos(cos_theta2); 

    % There are two general solutions for theta2 for most reachable points:
    % 1) Elbow-down configuration (positive theta2_base)
    % 2) Elbow-up configuration (negative theta2_base)
    theta2_rad_sol(1) = theta2_base; 
    theta2_rad_sol(2) = -theta2_base; 

    % 4. Calculate theta1 using atan2 formulation
    % This method is robust for determining the angle in all four quadrants.
    
    % First, find the absolute angle (phi) of the target point (x_target, y_target) from the origin.
    phi = atan2(y_target, x_target); 
    
    % Second, find the angle (beta) at the base formed by the first link and the line
    % connecting the base to the end-effector (D). This also uses the Law of Cosines.
    cos_beta = (L1^2 + D_squared - L2^2) / (2 * L1 * D);

    % Handle potential numerical errors for acos (input should be in [-1, 1])
    if cos_beta > 1
        cos_beta = 1;
    elseif cos_beta < -1
        cos_beta = -1;
    end
    beta = acos(cos_beta); % acos returns values in [0, pi]

    % Calculate the two solutions for theta1:
    % Solution 1: phi minus beta (typically for the elbow-down configuration)
    % Solution 2: phi plus beta (typically for the elbow-up configuration)
    theta1_rad_sol(1) = phi - beta; 
    theta1_rad_sol(2) = phi + beta; 

    % Normalize angles to be within -pi to pi radians (equivalent to -180 to 180 degrees)
    % This makes the output angles consistent and easier to interpret, especially for animations.
    theta1_rad_sol = wrapToPi(theta1_rad_sol);
    theta2_rad_sol = wrapToPi(theta2_rad_sol);
    
    reachable = true; % If we reached this point, the target is reachable
end