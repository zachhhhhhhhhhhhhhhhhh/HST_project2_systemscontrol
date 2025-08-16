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

% Initial states
x = zeros(4, T+1);
% With slight perturbation
x(:,1) = [0.951, ((0.01651 + (a2/a1)*(x(3)))/0.951)-0.0165, 0.31, 0];
u = zeros(2, T);
u(:,1) = [x(1,1)*(k0+x(2,1)), (a3 + a2*a3*a4 + a5)*(x(3,1))];
y = zeros(1, T+1);
y(:,1) = x(1);

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
L_Eigen = [0.02, 0.025, 0.03, 0.035];
L = place(A', C', L_eigen)'; 

% Observer Matrix
A_obs = A - L*C;
disp('Eigenvalues of observer: ');
disp(eig(A_obs))

% Observer initial condition (estimate of incremental state)
xhat = zeros(4, T+1);  

% Simulation loop
for t = 1:T
    % Current state
    x_t = [x1(t); x2(t); x3(t); x4(t)];       

    % Output of nonlinear system
    y_t = [x1(t)];          

    % Incremental measurement
    y_tilde = y_t - y_bar;

    % Control input
    u_tilde = -K * xhat(:, t);      

    % Apply absolute control to the nonlinear plant
    u1(t) = u_bar(1) + u_tilde(1);
    u2(t) = u_bar(2) + u_tilde(2);


    % Observer update
    xhat(:, t+1) = A * xhat(:, t) + B * u_tilde + L * (y_tilde - C * xhat(:, t) - D * u_tilde);       

    % Nonlinear system dynamics       
    x1(t+1) = x1(t) - (k0+x2(t))*x1(t) + u1(t);           % Glucose Concentration
    x2(t+1) = x2(t) - a1*x2(t) + a3*x3(t);                  % k(t)
    x3(t+1) = x3(t) - a3*x3(t) + a4*x4(t) + a6*x4(t) + u2(t); % i(t)
    x4(t+1) = x4(t) - a6*x4(t) +a5*x3(t);                   % i3(t)
end   

% Plotting
time = 0:T; 
figure; 
  
plot(time, x1, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(1), '--k', 'LineWidth', 2);  
ylabel('$x_1(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

plot(time, x2, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(2), '--k', 'LineWidth', 2);  
ylabel('$x_2(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

plot(time, x3, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(3), '--k', 'LineWidth', 2);  
ylabel('$x_3(t)$', 'Interpreter', 'latex', 'FontSize', 20); 

plot(time, x4, 'b', 'LineWidth', 1.5); hold on; 
yline(x_bar(4), '--k', 'LineWidth', 2);  
ylabel('$x_4(t)$', 'Interpreter', 'latex', 'FontSize', 20); 
ylim([-0.06 0.06]); 

time_u = 0:T-1;  % match u1
plot(time_u, u1, 'r', 'LineWidth', 1.5); hold on; 
yline(u_bar(1), '--k', 'LineWidth', 2); 
ylabel('$u_1(t)$', 'Interpreter', 'latex', 'FontSize', 20); 
xlabel('Time step', 'FontSize', 13);
plot(time_u, u2, 'b', 'LineWidth', 1.5);
yline(u_bar(2), '--k', 'LineWidth', 2);

sgtitle('Linear Observer-Based Control for Nonlinear System. Solid blue: states. Black dashed: equilibrium.', 'FontSize', 15);