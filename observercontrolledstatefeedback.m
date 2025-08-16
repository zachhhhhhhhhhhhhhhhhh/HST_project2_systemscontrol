
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

% Controller and observer gains
P_test = [0.7, 0.85, 0.9, 0.8];
K = place(A, B, P_test);
L_Eigen = [0.49,0.7225,0.81,0.64];
L = place(A', C', L_Eigen)';   

%% Desired equilibrium
x_f = [0;0;0;0];   

%% Simulation settings
T = 180;    

% Initial states
% True System
x = zeros(4, T+1);
x(:,1) = [0.951, ((0.01651 + (a2/a1)*(0.31)/0.951))-0.0165, 0.31, 0];
u = zeros(2, T+1);
u(:,1) = [x(1,1)*(k0+x(2,1)), (a3 + a2*a3*a4 + a5)*(x(3,1))];
y = zeros(1, T+1);
y(:, 1) = C*x(:,1) + D*u(:,1);

% Estimators 
x_hat = zeros(4, T+1);
x_hat(:, 1) = x_bar;
y_hat = zeros(1, T+1);
y_hat(:,1) = C*x_hat(:,1) + D*u(:,1);

% Initial outputs
y(1)     = C * x(:,1); 
y_hat(1) = C * x_hat(:,1);   

%% Simulate closed-loop system with observer-based control
for t = 1:T     
    % Pure regulation control: no extra input     
    u(:,t) = u_bar - K * ( x_hat(:,t) - x_f );       

    % True system     
    x(:,t+1) = A * x(:,t) + B * u(:,t);     
    y(t+1)        = C * x(:,t+1);       

    % Observer update     
    x_hat(:,t+1) = A * x_hat(:,t) + B * u(:,t) + L * ( y(t) - y_hat(t) );     
    y_hat(t+1)   = C * x_hat(:,t+1); 
end   

%% Figure 1: True vs Estimated States
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
title('Observer-Based Regulation: True vs. Estimated States', 'FontSize', 32, 'Interpreter', 'latex'); 
legend('FontSize', 24, 'Interpreter', 'latex', 'Location', 'best'); 
grid on; 
set(gca, 'FontSize', 24);   

%% Figure 2: True States vs Desired x_bar
figure; 
plot(0:T, x(1,:), '-', 'LineWidth', 3, 'DisplayName', '$x_1(t)$'); hold on; 
plot(0:T, x(2,:), '-', 'LineWidth', 3, 'DisplayName', '$x_2(t)$'); 
plot(0:T, x(3,:), '-', 'LineWidth', 3, 'DisplayName', '$x_3(t)$'); 
plot(0:T, x(4,:), '-', 'LineWidth', 3, 'DisplayName', '$x_4(t)$');   

% Plot x_bar lines 
yline(x_bar(1), '--', 'LineWidth', 2, 'DisplayName', '$\bar{x}_1$'); 
yline(x_bar(2), '--', 'LineWidth', 2, 'DisplayName', '$\bar{x}_2$'); 
yline(x_bar(3), '--', 'LineWidth', 2, 'DisplayName', '$\bar{x}_3$'); 
yline(x_bar(4), '--', 'LineWidth', 2, 'DisplayName', '$\bar{x}_4$');   

xlabel('Time Step $t$', 'FontSize', 28, 'Interpreter', 'latex'); 
ylabel('States vs. Target', 'FontSize', 28, 'Interpreter', 'latex'); 
title('Observer-Based Regulation: States Converging to $\bar{x}$', 'FontSize', 32, 'Interpreter', 'latex'); 
legend('FontSize', 24, 'Interpreter', 'latex', 'Location', 'best'); 
grid on; 
set(gca, 'FontSize', 24);