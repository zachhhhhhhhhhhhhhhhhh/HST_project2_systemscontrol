% Eigenvalue and Eigenvector Analysis for Glucose-Insulin Control System
% HST Project 2 - Systems Control

% Define the system matrix A for glucose-insulin model
A = [674/675 -0.9 0 0; 
    0 0.606 0.251 0; 
    0 0 0.606 2800; 
    0 0 3.15*10.^-8 -2799];

% Display the matrix A
fprintf('System Matrix A:\n');
disp(A);

% Calculate eigenvalues only
lambda = eig(A);
fprintf('Eigenvalues (lambda):\n');
disp(lambda);

% Calculate both eigenvalues and eigenvectors
[V, D] = eig(A);
fprintf('Eigenvalues (diagonal of D):\n');
disp(diag(D));

fprintf('Eigenvectors (columns of V):\n');
disp(V);

% Display each eigenvector with its corresponding eigenvalue
fprintf('\nDetailed Eigenanalysis:\n');
fprintf('========================\n');
for i = 1:length(lambda)
    fprintf('Eigenvalue %d: %.6f\n', i, lambda(i));
    fprintf('Corresponding Eigenvector %d:\n', i);
    fprintf('  [%.6f, %.6f, %.6f, %.6f]^T\n', V(:,i));
    fprintf('\n');
end

% Check stability (all eigenvalues should have negative real parts for stability)
fprintf('Stability Analysis:\n');
fprintf('==================\n');
real_parts = real(lambda);
if all(real_parts < 0)
    fprintf('System is STABLE (all eigenvalues have negative real parts)\n');
else
    fprintf('System is UNSTABLE (some eigenvalues have non-negative real parts)\n');
end

fprintf('Real parts of eigenvalues: ');
fprintf('%.6f  ', real_parts);
fprintf('\n');

% Display eigenvalues in scientific notation
fprintf('\nEigenvalues in scientific notation:\n');
for i = 1:length(lambda)
    fprintf('λ_%d = %.6e\n', i, lambda(i));
end

% Additional analysis
fprintf('\nAdditional Properties:\n');
fprintf('======================\n');
fprintf('Determinant of A: %.6e\n', det(A));
fprintf('Trace of A: %.6f\n', trace(A));
fprintf('Condition number: %.6e\n', cond(A));

% Save results to workspace
save('eigenanalysis_results.mat', 'A', 'lambda', 'V', 'D');
fprintf('\nResults saved to eigenanalysis_results.mat\n');
