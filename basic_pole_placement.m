% EIGENVALUE ASSIGNMENT WITHOUT CONTROL SYSTEM TOOLBOX
% HST Project 2 - Basic MATLAB implementation
% Manual controllability check and pole placement design

clear; clc;

% System matrices
A = [-306.8941178 0.01008483167 0 0; 
        0 0.606 0.142 0; 
        0 0.394 0.749 2800; 
        0 0 3.15*10.^-8 -2799];

B = [1 0;
     0 0;
     0 1;
     0 0];

fprintf('EIGENVALUE ASSIGNMENT - BASIC MATLAB IMPLEMENTATION\n');
fprintf('==================================================\n\n');

% 1. SYSTEM ANALYSIS
fprintf('1. SYSTEM ANALYSIS:\n');
fprintf('------------------\n');
fprintf('System: dx/dt = Ax + Bu\n');
fprintf('State dimension n = %d\n', size(A,1));
fprintf('Input dimension m = %d\n', size(B,2));

% Open-loop eigenvalues
lambda_ol = eig(A);
fprintf('\nOpen-loop eigenvalues:\n');
for i = 1:length(lambda_ol)
    fprintf('λ_%d = %10.6f', i, lambda_ol(i));
    if real(lambda_ol(i)) > 0
        fprintf(' (UNSTABLE)');
    else
        fprintf(' (STABLE)');
    end
    fprintf('\n');
end

% 2. MANUAL CONTROLLABILITY CHECK
fprintf('\n2. CONTROLLABILITY CHECK (Manual Implementation):\n');
fprintf('------------------------------------------------\n');

% Build controllability matrix manually: [B AB A²B A³B]
n = size(A, 1);
Ctrl = B;  % Start with B

fprintf('Building controllability matrix [B AB A²B A³B]...\n');

% Add AB, A²B, A³B, etc.
A_power = eye(n);  % A⁰ = I
for i = 1:(n-1)
    A_power = A_power * A;  % A^i
    Ctrl = [Ctrl, A_power * B];
end

fprintf('Controllability matrix size: %dx%d\n', size(Ctrl,1), size(Ctrl,2));

% Check rank manually
rank_ctrl = rank(Ctrl);
fprintf('Controllability matrix rank: %d\n', rank_ctrl);
fprintf('Required rank for full controllability: %d\n', n);

if rank_ctrl == n
    fprintf('✓ System is COMPLETELY CONTROLLABLE\n');
    fprintf('✓ All eigenvalues can be assigned\n');
    controllable = true;
else
    fprintf('✗ System is NOT completely controllable\n');
    fprintf('✗ Rank deficiency: %d\n', n - rank_ctrl);
    controllable = false;
end

% 3. SIMPLIFIED POLE PLACEMENT FOR MULTI-INPUT CASE
fprintf('\n3. CONTROLLER DESIGN (Simplified Approach):\n');
fprintf('------------------------------------------\n');

if ~controllable
    fprintf('Cannot proceed with pole placement - system not controllable\n');
    return;
end

% For multi-input systems, we'll use a simplified LQR-like approach
% without requiring the Control System Toolbox

fprintf('Using manual LQR-like design for multi-input system...\n');

% Design weighting matrices
Q = eye(n);          % State weighting
Q(2,2) = 100;        % Emphasize glucose-related states
Q(3,3) = 100;        % Emphasize insulin-related states
R = eye(size(B,2));  % Control effort weighting

fprintf('State weighting matrix Q:\n');
disp(Q);
fprintf('Control weighting matrix R:\n');
disp(R);

% Manual LQR solution using algebraic Riccati equation
% We'll solve it iteratively since we don't have 'lqr' function

fprintf('\nSolving algebraic Riccati equation manually...\n');

% Initial guess for P
P = Q;
max_iterations = 100;
tolerance = 1e-6;

for iter = 1:max_iterations
    P_old = P;
    
    % Riccati iteration: P = Q + A'P - PBR⁻¹B'P + PA
    try
        P = Q + A'*P + P*A - P*B*inv(R)*B'*P;
    catch
        fprintf('Matrix inversion failed. Using alternative approach...\n');
        break;
    end
    
    % Check convergence
    if norm(P - P_old, 'fro') < tolerance
        fprintf('Converged after %d iterations\n', iter);
        break;
    end
    
    if iter == max_iterations
        fprintf('Warning: Maximum iterations reached\n');
    end
end

% Calculate feedback gain: K = R⁻¹B'P
try
    K = inv(R) * B' * P;
    fprintf('✓ Feedback gain matrix calculated successfully\n');
catch
    fprintf('Failed to calculate feedback gain. Using alternative method...\n');
    
    % Alternative: Simple pole shifting approach
    fprintf('Using simplified pole shifting approach...\n');
    
    % For this specific system, design K to move poles to desired locations
    % This is a simplified approach for educational purposes
    
    % Target: move positive eigenvalues to negative values
    K = zeros(size(B,2), n);
    
    % Simple gains to stabilize the system
    K(1,1) = 1.0;    % Control glucose deviation
    K(1,2) = 10.0;   % Control insulin action
    K(2,3) = 0.01;   % Control remote insulin
    K(2,4) = 0.001;  % Control rapid insulin compartment
    
    fprintf('Using simplified gain matrix\n');
end

fprintf('\nCalculated feedback gain matrix K:\n');
disp(K);

% 4. CLOSED-LOOP ANALYSIS
fprintf('\n4. CLOSED-LOOP ANALYSIS:\n');
fprintf('------------------------\n');

A_cl = A - B * K;
lambda_cl = eig(A_cl);

fprintf('Closed-loop system matrix (A - BK):\n');
disp(A_cl);

fprintf('\nClosed-loop eigenvalues:\n');
all_stable = true;
for i = 1:length(lambda_cl)
    if imag(lambda_cl(i)) ~= 0
        fprintf('λ_%d = %8.4f %+8.4fi', i, real(lambda_cl(i)), imag(lambda_cl(i)));
    else
        fprintf('λ_%d = %8.4f', i, lambda_cl(i));
    end
    
    if real(lambda_cl(i)) < 0
        fprintf(' (STABLE)');
    else
        fprintf(' (UNSTABLE)');
        all_stable = false;
    end
    fprintf('\n');
end

% 5. DESIGN ASSESSMENT
fprintf('\n5. DESIGN ASSESSMENT:\n');
fprintf('--------------------\n');

if all_stable
    fprintf('✓ DESIGN SUCCESSFUL!\n');
    fprintf('✓ All closed-loop poles are stable\n');
    fprintf('✓ System will converge asymptotically to zero-state\n');
    
    % Calculate approximate settling time from dominant pole
    dominant_pole_real = max(real(lambda_cl));
    if dominant_pole_real < 0
        settling_time = 4 / abs(dominant_pole_real);
        fprintf('✓ Approximate settling time: %.2f minutes\n', settling_time);
    end
else
    fprintf('✗ DESIGN NEEDS IMPROVEMENT\n');
    fprintf('✗ Some poles are still unstable\n');
    fprintf('→ Try adjusting the gain matrix K\n');
end

% 6. CONTROL LAW IMPLEMENTATION
fprintf('\n6. CONTROL LAW:\n');
fprintf('--------------\n');
fprintf('Feedback control law: u = -Kx\n');
fprintf('where x = [x1; x2; x3; x4] is the state vector\n\n');

fprintf('Explicit control equations:\n');
fprintf('u1 = -(%.6f*x1 %+.6f*x2 %+.6f*x3 %+.6f*x4)\n', K(1,:));
fprintf('u2 = -(%.6f*x1 %+.6f*x2 %+.6f*x3 %+.6f*x4)\n', K(2,:));

% 7. MANUAL SIMULATION (Basic Integration)
fprintf('\n7. BASIC SIMULATION TEST:\n');
fprintf('------------------------\n');

% Simple simulation to verify stability
t_final = 60;  % 1 hour
dt = 0.5;      % 30 seconds
t = 0:dt:t_final;

% Initial condition
x0 = [50; 0.1; 0.05; 0];
x = x0;

fprintf('Testing with initial condition: [%.1f, %.2f, %.3f, %.1f]\n', x0);

% Simulate for a few time steps
for i = 1:min(10, length(t)-1)
    u = -K * x;
    x_dot = A_cl * x;
    x = x + dt * x_dot;
    
    if i <= 5  % Show first few steps
        fprintf('t=%.1f: x=[%.3f, %.4f, %.4f, %.3f], u=[%.3f, %.3f]\n', ...
                (i-1)*dt, x, u);
    end
end

if all(abs(x) < abs(x0))
    fprintf('✓ States are decreasing - system appears stable\n');
else
    fprintf('⚠ States may be growing - check controller design\n');
end

% 8. SAVE RESULTS
fprintf('\n8. SAVING RESULTS:\n');
fprintf('-----------------\n');

save('basic_controller_results.mat', 'A', 'B', 'K', 'A_cl', 'lambda_ol', 'lambda_cl', 'Q', 'R');
fprintf('✓ Results saved to basic_controller_results.mat\n');

% 9. SUMMARY AND RECOMMENDATIONS
fprintf('\n9. SUMMARY:\n');
fprintf('==========\n');
fprintf('Without Control System Toolbox, we implemented:\n');
fprintf('• Manual controllability matrix construction\n');
fprintf('• Simplified LQR-like controller design\n');
fprintf('• Basic stability analysis\n');
fprintf('• Simple simulation verification\n\n');

fprintf('Your feedback gain matrix K is:\n');
disp(K);

if all_stable
    fprintf('This controller stabilizes your glucose-insulin system!\n');
    fprintf('The zero-state equilibrium is now asymptotically stable.\n');
else
    fprintf('Controller needs refinement for full stabilization.\n');
    fprintf('Consider adjusting gain values manually.\n');
end

fprintf('\n===============================================\n');
fprintf('BASIC MATLAB POLE PLACEMENT DESIGN COMPLETE!\n');
fprintf('===============================================\n');
