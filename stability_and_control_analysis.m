% Stability vs Controllability Analysis
% HST Project 2 - Systems Control
% Question: Must equilibrium point be stable for control?

% Define the updated system matrix A
A = [-306.8941178 0.01008483167 0 0; 
        0 0.606 0.142 0; 
        0 0.394 0.749 2800; 
        0 0 3.15*10.^-8 -2799];

% Example control input matrix B (assuming single input)
B = [1 0;
     0 0;
     0 1;
    0 0];  % Adjust based on your actual system

fprintf('STABILITY vs CONTROLLABILITY ANALYSIS\n');
fprintf('=====================================\n\n');

% 1. EIGENVALUE ANALYSIS
fprintf('1. EIGENVALUE ANALYSIS:\n');
fprintf('-----------------------\n');
lambda = eig(A);
fprintf('Eigenvalues:\n');
for i = 1:length(lambda)
    fprintf('λ_%d = %.6f', i, lambda(i));
    if real(lambda(i)) > 0
        fprintf(' (UNSTABLE - positive real part)');
    elseif real(lambda(i)) < 0
        fprintf(' (STABLE - negative real part)');
    else
        fprintf(' (MARGINALLY STABLE - zero real part)');
    end
    fprintf('\n');
end

% Overall stability assessment
fprintf('\nSTABILITY ASSESSMENT:\n');
if all(real(lambda) < 0)
    fprintf('✓ System is ASYMPTOTICALLY STABLE\n');
    stable = true;
elseif any(real(lambda) > 0)
    fprintf('✗ System is UNSTABLE (has unstable modes)\n');
    stable = false;
else
    fprintf('? System is MARGINALLY STABLE\n');
    stable = false;
end

% 2. CONTROLLABILITY ANALYSIS
fprintf('\n2. CONTROLLABILITY ANALYSIS:\n');
fprintf('-----------------------------\n');

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


fprintf('YOUR SYSTEM STATUS:\n');
if stable && controllable
    fprintf('• Stable: YES ✓\n• Controllable: YES ✓\n');
    fprintf('→ Easy to control, maintains stability naturally\n');
elseif ~stable && controllable
    fprintf('• Stable: NO ✗\n• Controllable: YES ✓\n');
    fprintf('→ CAN BE CONTROLLED! Feedback can stabilize it\n');
elseif stable && ~controllable
    fprintf('• Stable: YES ✓\n• Controllable: NO ✗\n');
    fprintf('→ Some modes cannot be influenced by control\n');
else
    fprintf('• Stable: NO ✗\n• Controllable: NO ✗\n');
    fprintf('→ Cannot be controlled in all directions\n');
end

if ~stable
    fprintf('• The open-loop system is unstable\n');
    fprintf('• This is TYPICAL for biological systems\n');
    fprintf('• Feedback control is NECESSARY for stable operation\n');
    fprintf('• Many successful control systems start with unstable plants\n');
    fprintf('  (examples: aircraft, rockets, inverted pendulum)\n\n');
else
    fprintf('• The open-loop system is stable\n');
    fprintf('• Control can improve performance (faster response, disturbance rejection)\n\n');
end

if controllable
    fprintf('• All unstable modes can be stabilized with proper feedback\n');
    fprintf('• You can place closed-loop poles anywhere you want\n');
    fprintf('• Design techniques: pole placement, LQR, H∞, etc.\n');
else
    fprintf('• Some modes cannot be controlled\n');
    fprintf('• Check if uncontrollable modes are stable\n');
    fprintf('• May need additional actuators or different control strategy\n');
end

