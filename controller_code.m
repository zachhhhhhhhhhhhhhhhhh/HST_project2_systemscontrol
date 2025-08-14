
clear all; close all;

% Parameters
T = 60;                   % Total simulation time (in minutes)
k0 = 0;                   % Insulin-independent fractional removal rate
a1 = 0;                   % a1 - a6 parameters
a2 = 0;                 
a3 = 0;
a4 = 0;
a5 = 0;
a6 = 0;

% Equilibrium point (upright)
x_bar = [2;
         0;
         pi;
         0]; 
u_bar = [0;
         0];
y_bar = [2]; 

% Linearized system matrices at (x3 = pi)
A = [1, 1, 0, 0; 
     0, 1, -g, 0; 
     0, 0, 1, 1; 
     0, 0, -2*g, 1]; 
B = [1, 0; 
     0, 0;
     0, 1;
     0, 0]; 
C = [1, 0, 0, 0]; 
D = [0, 0];

% Controller and observer gains
K = place(A, B, [0.3, 0.4, 0.5, 0.6]); 
L = place(A', C', [0.02, 0.025, 0.03, 0.035])'; 

% Preallocate state variables
x1 = zeros(1, T+1); 
x2 = zeros(1, T+1); 
x3 = zeros(1, T+1); 
x4 = zeros(1, T+1); 

% Start close to the equilibrium
x1(1) = 2.75; 
x2(1) = 0; 
x3(1) = pi + 0.0025; 
x4(1) = 0; 

% Observer initial condition (estimate of incremental state)
xhat = zeros(4, T+1);  

% Control input
u = zeros(1, T+1); 

% Simulation loop
for t = 1:T
    % Current state
    x_t = [x1(t); x2(t); x3(t); x4(t)];       

    % Output of nonlinear system
    y_t = [x1(t); x3(t)];          

    % Compute incremental signals
    y_tilde = y_t - y_bar; 
    u_tilde = u(t) - u_bar;       

    % Control input at time t (uses current observer state)
    u_tilde = -K * xhat(:, t); 
    u(t) = u_bar + u_tilde;       

    % Observer update
    xhat(:, t+1) = A * xhat(:, t) + B * u_tilde + L * (y_tilde - C * xhat(:, t) - D * u_tilde);       

    % Nonlinear system dynamics
    d = 1 + sin(x3(t))^2;       
    x1(t+1) = x1(t) + x2(t); 
    x2(t+1) = x2(t) + (1/d) * (u(t) + sin(x3(t)) * x4(t)^2 - g * sin(x3(t)) * cos(x3(t))); 
    x3(t+1) = x3(t) + x4(t); 
    x4(t+1) = x4(t) + (1/d) * (-u(t) * cos(x3(t)) - sin(x3(t)) * cos(x3(t)) * x4(t)^2 + 2 * g * sin(x3(t))); 
end   

% Plotting
time = 0:T; 
figure; 

subplot(5, 1, 1);  
plot(time, x1, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(1), '--k', 'LineWidth', 2);  
ylabel('$x_1(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

subplot(5, 1, 2);  
plot(time, x2, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(2), '--k', 'LineWidth', 2);  
ylabel('$x_2(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

subplot(5, 1, 3);  
plot(time, x3, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(3), '--k', 'LineWidth', 2);  
ylabel('$x_3(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

subplot(5, 1, 4);  
plot(time, x4, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(4), '--k', 'LineWidth', 2);  
ylabel('$x_4(t)$', 'Interpreter', 'latex', 'FontSize', 20); 
ylim([-0.06 0.06]); 

subplot(5, 1, 5);  
plot(time, u, 'r', 'LineWidth', 1.5); hold on; 
yline(u_bar, '--k', 'LineWidth', 2); 
ylabel('$u(t)$', 'Interpreter', 'latex', 'FontSize', 20); 
xlabel('Time step', 'FontSize', 13); 

sgtitle('Linear Observer-Based Control for Nonlinear System. Solid blue: states. Black dashed: equilibrium.', 'FontSize', 15);