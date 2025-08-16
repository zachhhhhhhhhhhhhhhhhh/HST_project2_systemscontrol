
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

% Calculate and print the eigenvalues, checking for stability
eigenvalues = eig(A);
disp('Eigenvalues of the matrix A:');
disp(eigenvalues);

% Desired regulation point (regulation to zero)
xf = [0; 0; 0; 0];

% Compute steady-state input (ubar)
ubar = B \ ((eye(4) - A) * xf);
residual = norm((eye(4) - A) * xf - B * ubar);  % Optional check

% Check controllability matrix
ctrl = [B, A*B, A^2*B, A^3*B];
rankC = rank(ctrl);
disp('Rank of the controllability matrix:');
disp(rankC);

% Feedback design
P_test = [0.7, 0.85, 0.9, 0.8];
K = place(A, B, P_test); 
disp('K-gain:');
disp(K);

% Initial States
T = 180;
x = zeros(4, T+1);
% With slight perturbation
x(:,1) = [0.951; ((0.01651 + (a2/a1)*(x(3)))/0.951)-0.0165; 0.31; 0];
u = zeros(2, T);
u(:,1) = [x(1,1)*(k0+x(2,1)); (a3 + a2*a3*a4 + a5)*(x(3,1))];
y = zeros(1, T+1);
y(:,1) = x(1);

% Simulation loop
for t = 1:T
    u(:,t) = ubar - K * (x(:,t) - xf);
    x(:,t+1) = A * x(:,t) + B * u(:,t);
end

% Plotting results
figure;
plot(0:T, x(1,:), '-', 'LineWidth', 3, 'DisplayName', '$x_1(t)$');
hold on;
plot(0:T, x(2,:), '-', 'LineWidth', 3, 'DisplayName', '$x_2(t)$');
plot(0:T, x(3,:), '-', 'LineWidth', 3, 'DisplayName', '$x_3(t)$');
plot(0:T, x(4,:), '-', 'LineWidth', 3, 'DisplayName', '$x_4(t)$');
yline(x_bar(1), '--', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(x_bar(2), '--', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(x_bar(3), '--', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(x_bar(4), '--', 'LineWidth', 2, 'HandleVisibility', 'off');
xlabel('Time Step $t$', 'FontSize', 28, 'Interpreter', 'latex');
ylabel('State Components $x_i(t)$', 'FontSize', 28, 'Interpreter', 'latex');
title('State Convergence to $x_f$ from $x_0$ (Linearised Controller)', 'FontSize', 32, 'Interpreter', 'latex');
legend('FontSize', 24, 'Interpreter', 'latex', 'Location', 'best');
grid on;
set(gca, 'FontSize', 24);