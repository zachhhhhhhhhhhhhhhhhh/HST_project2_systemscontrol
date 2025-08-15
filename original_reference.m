clear; clc; close all;

function [A, B, C, D, x_bar, u_bar, y_bar, params] = original_reference(T)


    % Parameters
    % Simulation Time Boolean Check
    if nargin < 1
        T = 60;
    end

    params.k0 = 0.0165;              % Insulin-independent fractional removal rate
    params.a1 = 0.394;                   % a1 - a6 parameters
    params.a2 = 0.142;                 
    params.a3 = 0.251;
    params.a4 = 0.394;
    params.a5 = 3.15*10^-8;
    params.a6 = 2.8*10^3;

    % Equilibrium point (upright)
    x_bar1 = 1.08;
    x_bar3 = 0.3;
    x_bar2 = (params.a2*x_bar3)/params.a1;
    x_bar4 = (params.a5*x_bar3)/params.a6;
    x_bar = [x_bar1;
            x_bar2;
            x_bar3;
            x_bar4]; 
    u_bar = [x_bar1*(params.k0+x_bar2);
            (params.a3 + params.a2*params.a3*a4 + params.a5)*(x_bar3)];
    y_bar = x_bar1; 

    % Define your matrix A here
    A = [1-(params.k0+x_bar2), -(x_bar1), 0, 0; 
        0, 1-params.a1, params.a2, 0; 
        0, params.a4, 1-params.a3, params.a6; 
        0, 0, params.a5, 1-params.a6]; % Replace with your actual matrix definition

    B = [1, 0; 
     0, 0;
     0, 1;
     0, 0]; 
    
    C = [1, 0, 0, 0];
    
    D = [0, 0];

    if nargout > 0
        return;
    end

    % Calculate and print the eigenvalues
    eigenvalues = eig(A);
    disp('Eigenvalues of the matrix A:');
    disp(eigenvalues);

    % Initial states
    x1 = zeros(1, T+1);      % Inventory
    x2 = zeros(1, T+1);      % Degradation
    x3 = zeros(1, T+1);
    x4 = zeros(1,T+1);
    u1 = zeros(1, T);        % Control input
    u2 = zeros(1, T);
    x1(1) = 1.08;            % Initial inventory
    x2(1) = ((0.0165 + (a2/a1)*(0.25))/1.0)-0.0165;               % Initial degradation
    x3(1) = 0;
    x4(1) = 0;

    % Simulation loop
    for t = 1:T
        u1(t) = 0.0165 + (params.a2/params.a1)*(x3(t));   % IV Glucose input rate
        u2(t) = (params.a3 + params.a2*params.a3*params.a4 + params.a5)*(x3(t));                  % IV Insulin input rate
        x1(t+1) = x1(t) - (params.k0+x2(t))*x1(t) + u1(t);           % Glucose Concentration
        x2(t+1) = x2(t) - params.a1*x2(t) + params.a3*x3(t);                  % k(t)
        x3(t+1) = x3(t) - params.a3*x3(t) + params.a4*x4(t) + params.a6*x4(t) + u2(t); % i(t)
        x4(t+1) = x4(t) - params.a6*x4(t) + params.a5*x3(t);                   % i3(t)
    end

    % Plotting
    figure;
    subplot(1, 6, 1);
    plot(0:T, x1, 'b-', 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('[Glucose] $x_1(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('[Glucose] Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;

    subplot(1, 6, 2);
    plot(0:T, x2, 'Color', [1 0.4 0], 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('k(t) $x_2(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('k(t) Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;

    subplot(1, 6, 3);
    plot(0:T, x3, 'Color', [1 0.4 0], 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('i(t) $x_3(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('i(t) Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;

    subplot(1, 6, 4);
    plot(0:T, x4, 'Color', [1 0.4 0], 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('i3(t) $x_4(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('i3(t) Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;

    subplot(1, 6, 5);
    plot(1:T, u1, 'Color', [1 0.4 0], 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('IV Glucose $u1(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('IV Glucose Input Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;

    subplot(1, 6, 6);
    plot(1:T, u2,'Color', [1 0.4 0], 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('IV Insulin $u2(t)$', 'Interpreter', 'latex', 'FontSize', 16);
    title('IV Insulin Over Time', 'Interpreter', 'latex', 'FontSize', 16);
    grid on;


end