%
% Odometry Comparison: With vs Without Compensation
%

clear;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

WHEEL_BASE = 53;                    % [mm]
MM_PER_PULSE = 15.3*pi/600;         % [mm/pulse]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOAD ENCODER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ENC = load('khepera_circle.txt');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEST DIFFERENT SAMPLING RATES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sampling_factors = [1 2 5 10];

for sf = sampling_factors

    fprintf('\n=====================================\n');
    fprintf('Sampling every %d steps\n', sf);
    fprintf('=====================================\n');

    Dr = ENC(1:sf:end,2) * MM_PER_PULSE;
    Dl = ENC(1:sf:end,1) * MM_PER_PULSE;
    N = length(Dr);

    % Run without compensation
    [X_no, Y_no, A_no] = run_odometry(Dr, Dl, WHEEL_BASE, 0);

    % Run with compensation
    [X_comp, Y_comp, A_comp] = run_odometry(Dr, Dl, WHEEL_BASE, 1);

    % Compute differences
    diff_pos = sqrt((X_no - X_comp).^2 + (Y_no - Y_comp).^2);
    diff_ang = abs(A_no - A_comp);

    fprintf('Final position difference: %.6f mm\n', diff_pos(end));
    fprintf('Maximum position difference: %.6f mm\n', max(diff_pos));
    fprintf('RMSE position difference: %.6f mm\n', sqrt(mean(diff_pos.^2)));
    fprintf('Final angle difference: %.6f rad\n', diff_ang(end));

    % Plot comparison
    figure;
    plot(X_no, Y_no, 'b', 'LineWidth', 1.5); hold on;
    plot(X_comp, Y_comp, 'r--', 'LineWidth', 1.5);
    legend('Without Compensation','With Compensation');
    title(['Path Comparison (Sampling = ' num2str(sf) ')']);
    xlabel('X [mm]');
    ylabel('Y [mm]');
    axis equal;
    grid on;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ODOMETRY FUNCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X, Y, A] = run_odometry(Dr, Dl, WHEEL_BASE, COMP_TERM)

N = length(Dr);

X = zeros(N,1);
Y = zeros(N,1);
A = zeros(N,1);

% Initial state
X(1) = 0;
Y(1) = 0;
A(1) = 90*pi/180;

for kk = 2:N

    % Wheel increments
    dDr = Dr(kk) - Dr(kk-1);
    dDl = Dl(kk) - Dl(kk-1);

    % Relative motion
    dD = (dDr + dDl)/2;
    dA = (dDr - dDl)/WHEEL_BASE;

    theta_mid = A(kk-1) + dA/2;

    dX = dD * cos(theta_mid);
    dY = dD * sin(theta_mid);

    % Compensation term (Wang 1988)
    if COMP_TERM == 1
        if abs(dA) > 1e-6
            factor = sin(dA/2)/(dA/2);
            dX = dX * factor;
            dY = dY * factor;
        end
    end

    % Update state
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = A(kk-1) + dA;

end

end
%
% Odometry Comparison: With vs Without Compensation
%

clear;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

WHEEL_BASE = 53;                    % [mm]
MM_PER_PULSE = 15.3*pi/600;         % [mm/pulse]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOAD ENCODER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ENC = load('khepera_circle.txt');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TEST DIFFERENT SAMPLING RATES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sampling_factors = [1 2 5 10];

for sf = sampling_factors

    fprintf('\n=====================================\n');
    fprintf('Sampling every %d steps\n', sf);
    fprintf('=====================================\n');

    Dr = ENC(1:sf:end,2) * MM_PER_PULSE;
    Dl = ENC(1:sf:end,1) * MM_PER_PULSE;
    N = length(Dr);

    % Run without compensation
    [X_no, Y_no, A_no] = run_odometry(Dr, Dl, WHEEL_BASE, 0);

    % Run with compensation
    [X_comp, Y_comp, A_comp] = run_odometry(Dr, Dl, WHEEL_BASE, 1);

    % Compute differences
    diff_pos = sqrt((X_no - X_comp).^2 + (Y_no - Y_comp).^2);
    diff_ang = abs(A_no - A_comp);

    fprintf('Final position difference: %.6f mm\n', diff_pos(end));
    fprintf('Maximum position difference: %.6f mm\n', max(diff_pos));
    fprintf('RMSE position difference: %.6f mm\n', sqrt(mean(diff_pos.^2)));
    fprintf('Final angle difference: %.6f rad\n', diff_ang(end));

    % Plot comparison
    figure;
    plot(X_no, Y_no, 'b', 'LineWidth', 1.5); hold on;
    plot(X_comp, Y_comp, 'r--', 'LineWidth', 1.5);
    legend('Without Compensation','With Compensation');
    title(['Path Comparison (Sampling = ' num2str(sf) ')']);
    xlabel('X [mm]');
    ylabel('Y [mm]');
    axis equal;
    grid on;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ODOMETRY FUNCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X, Y, A] = run_odometry(Dr, Dl, WHEEL_BASE, COMP_TERM)

N = length(Dr);

X = zeros(N,1);
Y = zeros(N,1);
A = zeros(N,1);

% Initial state
X(1) = 0;
Y(1) = 0;
A(1) = 90*pi/180;

for kk = 2:N

    % Wheel increments
    dDr = Dr(kk) - Dr(kk-1);
    dDl = Dl(kk) - Dl(kk-1);

    % Relative motion
    dD = (dDr + dDl)/2;
    dA = (dDr - dDl)/WHEEL_BASE;

    theta_mid = A(kk-1) + dA/2;

    dX = dD * cos(theta_mid);
    dY = dD * sin(theta_mid);

    % Compensation term (Wang 1988)
    if COMP_TERM == 1
        if abs(dA) > 1e-6
            factor = sin(dA/2)/(dA/2);
            dX = dX * factor;
            dY = dY * factor;
        end
    end

    % Update state
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = A(kk-1) + dA;

end

end
