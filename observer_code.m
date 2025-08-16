% Parameters
k0 = 0.0165;              % Insulin-independent fractional removal rate
a1 = 0.394;                   % a1 - a6 parameters
a2 = 0.142;                 
a3 = 0.251;
a4 = 0.394;
a5 = (3.15*10^-8)*(7.5*10^-6);
a6 = 2.8*10^3*(7.5*10^-6);

% Equilibrium point (upright)
x_bar1 = 0.95;
x_bar3 = 0.3;
x_bar2 = (a2*x_bar3)/a1;
x_bar4 = (a5*x_bar3)/a6;
x_bar = [x_bar1;
         x_bar2;
         x_bar3;
         x_bar4]; 

u_bar = [x_bar1*(k0+x_bar2);
         (a3 + a2*a3*a4 + a5)*(x_bar3)];
y_bar = x_bar1; 

% A and B Matrix Formation (Jacobian Linearisation Matrix)
A = [1-(k0+x_bar2), -(x_bar1), 0, 0; 
     0, 1-a1, a2, 0; 
     0, a4, 1-a3, a6; 
     0, 0, a5, 1-a6];

B = [1, 0; 
     0, 0;
     0, 1;
     0, 0];

C = [1, 0, 0, 0];

D = [0, 0];


% Calculate and print the eigenvalues
eigenvalues = eig(A);
disp('Eigenvalues of the matrix A:');
disp(eigenvalues);

% Observability check
O = obsv(A,C);
assert(rank(O) == size(A,1), "System not fully observable");

% Controllability Check
n = size(A, 1);
Ctrl = B;
for i = 1:n-1
    Ctrl = [Ctrl A^i*B];
end
Ctrl_rank = rank(Ctrl);
disp('Controllability Matrix:');
disp(Ctrl);
disp('Controllability Matrix Rank:');
disp(Ctrl_rank);

% Controller and observer gains
P_test = [0.7, 0.85, 0.9, 0.8];
K = place(A, B, P_test);
L_Eigen = [0.49,0.3,0.81,0.64];
L = place(A', C', L_Eigen)'; 

% Observer Matrix
A_obs = A - L*C;
disp('Eigenvalues of observer: ');
disp(eig(A_obs))

% Initial states
% True System
x = zeros(4, T+1);
x(:,1) = [0.951, ((0.01651 + (a2/a1)*(x(3)))/0.951)-0.0165, 0.31, 0];
u = zeros(2, T+1);
u(:,1) = [x(1,1)*(k0+x(2,1)), (a3 + a2*a3*a4 + a5)*(x(3,1))];
y = zeros(1, T+1);
y(:, 1) = C*x(:,1) + D*u(:,1);

% Estimators 
x_hat = zeros(4, T+1);
x_hat(:, 1) = x_bar;
y_hat = zeros(1, T+1);
y_hat(:,1) = C*x_hat(:,1) + D*u(:,1);

%% Simulate true system + observer 
for t = 1:T 
    % True system 
    x(:,t+1) = A * x(:,t) + B * u(:,t); 
    y(t+1) = C * x(:,t+1) + D * u(:,t+1); 
    % Observer 
    x_hat(:,t+1) = A * x_hat(:,t) + B * u(:,t) + L * ( y(t) - y_hat(t) );
    y_hat(t+1) = C * x_hat(:,t+1) + D * u(:,t+1); 
end 

% Plot: True vs. Estimated States figure
figure;
plot(0:T, x(1,:), '-', 'LineWidth', 3, 'DisplayName', 'True $x_1(t)$'); hold on;
plot(0:T, x(2,:), '-', 'LineWidth', 3, 'DisplayName', 'True $x_2(t)$');
plot(0:T, x(3,:), '-', 'LineWidth', 3, 'DisplayName', 'True $x_3(t)$');
plot(0:T, x(4,:), '-', 'LineWidth', 3, 'DisplayName', 'True $x_4(t)$');
plot(0:T, x_hat(1,:), '--', 'LineWidth', 3, 'DisplayName', 'Estimated $\hat{x}_1(t)$');
plot(0:T, x_hat(2,:), '--', 'LineWidth', 3, 'DisplayName', 'Estimated $\hat{x}_2(t)$');
plot(0:T, x_hat(3,:), '--', 'LineWidth', 3, 'DisplayName', 'Estimated $\hat{x}_3(t)$');
plot(0:T, x_hat(4,:), '--', 'LineWidth', 3, 'DisplayName', 'Estimated $\hat{x}_4(t)$');

xlabel('Time Step $t$', 'FontSize', 28, 'Interpreter', 'latex');
ylabel('States and Estimates', 'FontSize', 28, 'Interpreter', 'latex');
title('Observer: True vs. Estimated States', 'FontSize', 32, 'Interpreter', 'latex');
legend('FontSize', 24, 'Interpreter', 'latex', 'Location', 'best');
grid on;
set(gca, 'FontSize', 24);