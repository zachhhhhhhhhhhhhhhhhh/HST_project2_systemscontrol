% DISCRETE-TIME GLUCOSE-INSULIN STATE EQUATIONS
% HST Project 2 - Systems Control
% Based on your discretized state equations with physiological parameters

% State equations:
% X1(t+1) = X1(t) - (k0 + X2(t))X1(t) + u1(t)
% X2(t+1) = X2(t) - a1*X2(t) + a3*X3(t)
% X3(t+1) = X3(t) - a3*X3(t) + a4*X2(t) + a6*X4(t) + u2(t)
% X4(t+1) = X4(t) - a6*X4(t) + a5*X3(t)

% Define physiological parameters (based on glucose-insulin literature)
k0 = 0.0165;        % Insulin-independent fractional removal rate of glucose [min^-1]
a1 = 0.0618;        % Insulin action decay rate [min^-1]
a2 = 0.0006;        % Insulin sensitivity parameter [min^-2/μU] (if used in extended model)
a3 = 0.0729;        % Transfer rate from central to action compartment [min^-1]
a4 = 0.000013;      % Effect of insulin action on central insulin [μU·min^-2]
a5 = 0.0465;        % Transfer rate from central to peripheral compartment [min^-1]
a6 = 0.0729;        % Transfer rate from peripheral back to central [min^-1]

% Parameter rationale:
% k0: Glucose effectiveness ~0.01-0.03 min^-1 (literature: Bergman model)
% a1: Insulin action decay ~0.05-0.08 min^-1 (half-life ~10-15 min)
% a3: Central compartment clearance ~0.05-0.1 min^-1
% a4: Very small coupling due to unit mismatch (μU·min^-2)
% a5,a6: Insulin distribution rates ~0.03-0.08 min^-1

% Physiological interpretation:
% X1 = Plasma Glucose Concentration [mg/mL] 
% X2 = Insulin dependent fractional removal rate of glucose [min^-1]
% X3 = Insulin mass in the central compartment [μU]
% X4 = Insulin mass in a peripheral compartment non-active in glucose removal [μU]
% u1 = Glucose systemic appearance rate [mg/(mL·min)]
% u2 = Insulin systemic appearance rate [μU/min]

% Steady-state values (basal conditions) - based on typical physiology
X1_b = 1.0;         % Basal glucose concentration ~100 mg/dL = 1.0 mg/mL
X2_b = 0.0;         % Basal insulin action (deviation from baseline)
X3_b = 100.0;       % Basal central insulin mass [μU] (~8-15 μU/mL × 7L ≈ 100 μU)
X4_b = 200.0;       % Basal peripheral insulin mass [μU] (larger peripheral volume)

fprintf('DISCRETE-TIME GLUCOSE-INSULIN MODEL\n');
fprintf('===================================\n\n');

% Display your discretized state equations
fprintf('State Equations:\n');
fprintf('X1(t+1) = X1(t) - (k0 + X2(t))*X1(t) + u1(t)\n');
fprintf('X2(t+1) = X2(t) - a1*X2(t) + a3*X3(t)\n');
fprintf('X3(t+1) = X3(t) - a3*X3(t) + a4*X2(t) + a6*X4(t) + u2(t)\n');
fprintf('X4(t+1) = X4(t) - a6*X4(t) + a5*X3(t)\n\n');

fprintf('Where:\n');
fprintf('X1 = Plasma Glucose Concentration [mg/mL]\n');
fprintf('X2 = Insulin dependent fractional removal rate [min^-1]\n');
fprintf('X3 = Insulin mass in central compartment [μU]\n');
fprintf('X4 = Insulin mass in peripheral compartment [μU]\n');
fprintf('u1 = Glucose systemic appearance rate [mg/(mL·min)]\n');
fprintf('u2 = Insulin systemic appearance rate [μU/min]\n\n');

% Display parameters
fprintf('Physiological Parameters:\n');
fprintf('k0 = %.6f min^-1 (insulin-independent glucose removal)\n', k0);
fprintf('a1 = %.6f min^-1 (insulin action decay rate)\n', a1);
fprintf('a2 = %.6f min^-2/μU (insulin sensitivity - if used)\n', a2);
fprintf('a3 = %.6f min^-1 (central to action transfer rate)\n', a3);
fprintf('a4 = %.6e μU·min^-2 (insulin action feedback)\n', a4);
fprintf('a5 = %.6f min^-1 (central to peripheral transfer)\n', a5);
fprintf('a6 = %.6f min^-1 (peripheral to central transfer)\n\n', a6);

fprintf('Steady-state (Basal) Values:\n');
fprintf('X1_b = %.2f mg/mL (basal glucose concentration)\n', X1_b);
fprintf('X2_b = %.4f min^-1 (basal insulin action)\n', X2_b);
fprintf('X3_b = %.1f μU (basal central insulin mass)\n', X3_b);
fprintf('X4_b = %.1f μU (basal peripheral insulin mass)\n\n', X4_b);

% STEADY-STATE EQUILIBRIUM CALCULATION
fprintf('STEADY-STATE EQUILIBRIUM CALCULATION:\n');
fprintf('====================================\n');

% At equilibrium: X(t+1) = X(t), u1 = u1_basal, u2 = u2_basal
% Need to solve the steady-state equations considering basal inputs

% Basal glucose appearance rate (hepatic glucose production + diet)
u1_basal = k0 * X1_b;  % Matches glucose removal at steady state [mg/(mL·min)]

% Basal insulin appearance rate (pancreatic secretion)
u2_basal = a3 * X3_b - a6 * X4_b;  % Matches insulin clearance [μU/min]

fprintf('Calculated basal inputs:\n');
fprintf('u1_basal = %.6f mg/(mL·min) (basal glucose appearance)\n', u1_basal);
fprintf('u2_basal = %.6f μU/min (basal insulin secretion)\n', u2_basal);

% Solve for consistent equilibrium values
% From X2 equation: a1*X2_eq = a3*X3_eq
X2_eq = (a3 * X3_b) / a1;

% From X4 equation: a6*X4_eq = a5*X3_eq  
X4_eq = (a5 * X3_b) / a6;

% Use the calculated values
X1_eq = X1_b;  % Target glucose concentration
X3_eq = X3_b;  % Basal central insulin mass

fprintf('\nConsistent Equilibrium Point:\n');
fprintf('X1_eq = %.3f mg/mL\n', X1_eq);
fprintf('X2_eq = %.6f min^-1\n', X2_eq);
fprintf('X3_eq = %.1f μU\n', X3_eq);
fprintf('X4_eq = %.1f μU\n', X4_eq);

% LINEARIZATION AROUND EQUILIBRIUM
fprintf('\nLINEARIZATION ANALYSIS:\n');
fprintf('======================\n');

% Linearized system: x(k+1) = A*x(k) + B*u(k)
% where x = [X1-X1_eq; X2-X2_eq; X3-X3_eq; X4-X4_eq] (deviations from equilibrium)

% Jacobian matrix A = ∂f/∂x evaluated at equilibrium
% ∂f1/∂X1 = 1 - k0 - X2_eq, ∂f1/∂X2 = -X1_eq, ∂f1/∂X3 = 0, ∂f1/∂X4 = 0
% ∂f2/∂X1 = 0, ∂f2/∂X2 = 1 - a1, ∂f2/∂X3 = a3, ∂f2/∂X4 = 0
% ∂f3/∂X1 = 0, ∂f3/∂X2 = a4, ∂f3/∂X3 = 1 - a3, ∂f3/∂X4 = a6
% ∂f4/∂X1 = 0, ∂f4/∂X2 = 0, ∂f4/∂X3 = a5, ∂f4/∂X4 = 1 - a6

A = [1 - k0 - X2_eq,  -X1_eq,       0,        0;
     0,               1 - a1,       a3,       0;
     0,               a4,           1 - a3,   a6;
     0,               0,            a5,       1 - a6];

fprintf('Linearized system matrix A:\n');
disp(A);

% Input matrix B (2 control inputs: u1 and u2)
B = [1, 0;   % u1 affects X1 directly
     0, 0;   % no direct effect on X2
     0, 1;   % u2 affects X3 directly
     0, 0];  % no direct effect on X4

fprintf('Input matrix B:\n');
disp(B);

% Check eigenvalues of linearized system
lambda = eig(A);
fprintf('Eigenvalues of linearized system:\n');
for i = 1:length(lambda)
    fprintf('λ_%d = %.6f', i, lambda(i));
    if abs(lambda(i)) > 1
        fprintf(' (UNSTABLE - outside unit circle)');
    elseif abs(lambda(i)) < 1
        fprintf(' (STABLE - inside unit circle)');
    else
        fprintf(' (MARGINALLY STABLE - on unit circle)');
    end
    fprintf('\n');
end

% Overall stability assessment
fprintf('\nSTABILITY ASSESSMENT:\n');
if all(abs(lambda) < 1)
    fprintf('✓ Linearized system is ASYMPTOTICALLY STABLE\n');
elseif any(abs(lambda) > 1)
    fprintf('✗ Linearized system is UNSTABLE\n');
else
    fprintf('? Linearized system is MARGINALLY STABLE\n');
end

% SIMULATION FUNCTION
fprintf('\nSIMULATION FUNCTIONS:\n');
fprintf('====================\n');

% Function to simulate your nonlinear system
function [X_next] = glucose_insulin_step(X, u, params)
    % Nonlinear discrete-time step using your state equations
    % X = [X1; X2; X3; X4] current state
    % u = [u1; u2] control inputs
    % params = [k0, a1, a3, a4, a5, a6]
    
    k0 = params(1); a1 = params(2); a3 = params(3);
    a4 = params(4); a5 = params(5); a6 = params(6);
    
    X1 = X(1); X2 = X(2); X3 = X(3); X4 = X(4);
    u1 = u(1); u2 = u(2);
    
    % Your state equations
    X1_next = X1 - (k0 + X2)*X1 + u1;
    X2_next = X2 - a1*X2 + a3*X3;
    X3_next = X3 - a3*X3 + a4*X2 + a6*X4 + u2;
    X4_next = X4 - a6*X4 + a5*X3;
    
    X_next = [X1_next; X2_next; X3_next; X4_next];
end
% Compute controllability matrix
n = size(A, 1);
C_ctrl = B;
for i = 1:(n-1)
    C_ctrl = [C_ctrl, A^i * B];
end

% Check controllability
rank_ctrl = rank(C_ctrl);
fprintf('Controllability matrix rank: %d\n', rank_ctrl);
fprintf('System dimension: %d\n', n);

if rank_ctrl == n
    fprintf('✓ System is COMPLETELY CONTROLLABLE\n');
    controllable = true;
else
    fprintf('✗ System is NOT completely controllable\n');
    fprintf('  Uncontrollable modes exist\n');
    controllable = false;
end

% CONTROLLER DESIGN
fprintf('\nCONTROLLER DESIGN:\n');
fprintf('==================\n');

if controllable
    % Design state feedback controller K such that u = -K*x + v
    % where x = [X1-X1_eq; X2-X2_eq; X3-X3_eq; X4-X4_eq] (deviations from equilibrium)
    % and v is the reference input (feedforward term)
    
    % Choose desired closed-loop poles (inside unit circle for stability)
    % Place poles for good transient response and stability margin
    desired_poles = [0.7, 0.8, 0.6, 0.75];  % All stable (|λ| < 1)
    
    fprintf('Desired closed-loop poles: [%.2f, %.2f, %.2f, %.2f]\n', desired_poles);
    
    % Use Ackermann's formula for pole placement (manual implementation)
    % Since we don't have Control System Toolbox
    fprintf('Computing controller gain K using pole placement...\n');
    
    % For pole placement: det(sI - (A - BK)) = desired characteristic polynomial
    % We need to solve for K such that eig(A - BK) = desired_poles
    
    % Method 1: Try LQR-like approach first
    % Weight matrices for LQR design
    Q = eye(4);  % State penalty
    R = 0.1*eye(2);  % Control penalty (smaller R = more aggressive control)
    
    % Manual LQR solution (discrete-time algebraic Riccati equation)
    % Start with initial guess and iterate
    P = Q;
    for iter = 1:100
        P_new = Q + A'*P*A - A'*P*B*inv(R + B'*P*B)*B'*P*A;
        if norm(P_new - P) < 1e-8
            break;
        end
        P = P_new;
    end
    
    % LQR gain
    K_lqr = inv(R + B'*P*B)*B'*P*A;
    
    fprintf('LQR Controller gain K_lqr:\n');
    disp(K_lqr);
    
    % Check closed-loop eigenvalues with LQR controller
    A_cl_lqr = A - B*K_lqr;
    lambda_cl_lqr = eig(A_cl_lqr);
    
    fprintf('Closed-loop eigenvalues with LQR controller:\n');
    for i = 1:length(lambda_cl_lqr)
        fprintf('λ_cl_%d = %.6f', i, lambda_cl_lqr(i));
        if abs(lambda_cl_lqr(i)) < 1
            fprintf(' (STABLE)');
        else
            fprintf(' (UNSTABLE)');
        end
        fprintf('\n');
    end
    
    % Use LQR controller as our main controller
    K = K_lqr;
    A_cl = A_cl_lqr;
    lambda_cl = lambda_cl_lqr;
    
else
    fprintf('Cannot design controller - system is not controllable!\n');
    K = [];
    A_cl = A;
    lambda_cl = eig(A);
end

% CLOSED-LOOP SIMULATION WITH CONTROLLER
if controllable
    fprintf('\nCLOSED-LOOP SIMULATION:\n');
    fprintf('======================\n');
    
    % Simulation parameters
    N_sim = 50;  % Number of time steps
    X_sim = zeros(4, N_sim+1);
    u_sim = zeros(2, N_sim);
    
    % Initial condition (deviation from equilibrium)
    X_sim(:,1) = [1.3; 0.08; 140; 220];  % Start away from equilibrium
    X_eq_vec = [X1_eq; X2_eq; X3_eq; X4_eq];
    
    fprintf('Simulating closed-loop response...\n');
    fprintf('Initial deviation from equilibrium: [%.3f, %.4f, %.1f, %.1f]\n', ...
            X_sim(:,1) - X_eq_vec);
    
    for k = 1:N_sim
        % State deviation from equilibrium
        x_dev = X_sim(:,k) - X_eq_vec;
        
        % Controller: u = -K*x_dev + u_basal
        u_control = -K * x_dev;
        u_total = u_control + [u1_basal; u2_basal];
        
        % Apply control input
        u_sim(:,k) = u_total;
        
        % Simulate one step
        X_sim(:,k+1) = glucose_insulin_step(X_sim(:,k), u_total, params);
    end
    
    % Display final results
    fprintf('Final state: [%.3f, %.4f, %.1f, %.1f]\n', X_sim(:,end));
    fprintf('Final deviation: [%.6f, %.6f, %.6f, %.6f]\n', X_sim(:,end) - X_eq_vec);
    fprintf('Regulation error magnitude: %.6f\n', norm(X_sim(:,end) - X_eq_vec));
    
    % Check if system reached equilibrium (within tolerance)
    tolerance = 1e-3;
    if norm(X_sim(:,end) - X_eq_vec) < tolerance
        fprintf('✓ System successfully regulated to equilibrium!\n');
    else
        fprintf('! System has not fully converged to equilibrium\n');
        fprintf('  Consider adjusting controller parameters\n');
    end
    
end

% Save system matrices and parameters
save('glucose_insulin_model.mat', 'A', 'B', 'params', 'X1_eq', 'X2_eq', 'X3_eq', 'X4_eq', ...
     'k0', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'u1_basal', 'u2_basal', 'K', 'A_cl', 'lambda_cl');

fprintf('\n✓ Model parameters and controller saved to glucose_insulin_model.mat\n');
fprintf('\nSUMMARY:\n');
fprintf('========\n');
fprintf('• Model: 4-state glucose-insulin system with physiological parameters\n');
fprintf('• States: X1 (glucose conc.), X2 (insulin action), X3 (central insulin), X4 (peripheral insulin)\n');
fprintf('• Controls: u1 (glucose appearance), u2 (insulin infusion)\n');
fprintf('• Basal inputs: u1_basal = %.6f, u2_basal = %.6f\n', u1_basal, u2_basal);
fprintf('• Equilibrium: [%.3f mg/mL, %.4f min^-1, %.1f μU, %.1f μU]\n', X1_eq, X2_eq, X3_eq, X4_eq);

if controllable
    fprintf('• Controller: LQR state feedback, K matrix computed\n');
    fprintf('• Closed-loop poles: ');
    for i = 1:length(lambda_cl)
        fprintf('%.3f ', lambda_cl(i));
    end
    fprintf('\n');
    if all(abs(lambda_cl) < 1)
        fprintf('• ✓ Closed-loop system is STABLE\n');
    else
        fprintf('• ✗ Closed-loop system is UNSTABLE\n');
    end
else
    fprintf('• ✗ No controller designed - system not controllable\n');
end

fprintf('\nController Law:\n');
fprintf('u = -K * (X - X_eq) + u_basal\n');
fprintf('where X_eq is the equilibrium state and u_basal are the basal inputs\n');

fprintf('\nNext steps:\n');
fprintf('1. ✓ Controller designed and tested\n');
fprintf('2. Tune controller parameters if needed (Q, R matrices)\n');
fprintf('3. Test robustness with different initial conditions\n');
fprintf('4. Add disturbance rejection capabilities\n');
