% Dead Reckoning with Snowhite Steed-drive Robot
%
% Björn Åstrand
% Version 5

clear all;
close all;
% The path plot can be toggled on and off to speed up the path plotting. 
% However, for debugging, you have the option to plot the path in the loop. 
% Otherwise, this is done after the loop is finished.
PLOT_PATH = 0; % [0, 1] 

% %%% Snowhite settings 
L = 680;               % [mm] wheelbase
T = 0.050;             % [sec] Sampling time

% Load encoder values
CONTROL = load('snowhite.txt');

% Init Robot Position, i.e. from Ground Truth
X(1) = CONTROL(1,3);
Y(1) = CONTROL(1,4);
A(1) = CONTROL(1,5);
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

% Run until no more values are available, i.e. speed and steering angle
N = max(size(CONTROL)); 
disp('Calculating ...');
for kk=2:N,  
    % Read current control values
    v = CONTROL(kk-1,1); % Speed of the steering wheel
    a = CONTROL(kk-1,2); % Angle of the steering wheel
    
    % Change of relative movements
    dD = v * cos(a) * T;
    dA = (v * sin(a) * T) / L;
    
    % Calculate the change in X and Y (World co-ordinates)
    theta_mid = A(kk-1) + dA/2;

    dX = dD * cos(theta_mid);
    dY = dD * sin(theta_mid);

    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = mod(A(kk-1) + dA, 2*pi);
    
    % Uncertainty in state variables at time k-1 [3x3]
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];      
    
    % memory of the rror
    % J_Xk-1: Partial derivatives w.r.t state (x, y, theta)
    Axya = [1, 0, -dD * sin(theta_mid);
            0, 1,  dD * cos(theta_mid);
            0, 0,  1];

    % J_vaT: Partial derivatives w.r.t inputs (v, alpha)
    % These are derived from the state update equations above.
    % Partial derivatives w.r.t linear velocity:
    df_dv = [T*cos(a)*cos(theta_mid) - (dD*T*sin(a)/(2*L))*sin(theta_mid);
             T*cos(a)*sin(theta_mid) + (dD*T*sin(a)/(2*L))*cos(theta_mid);
             (T*sin(a))/L];
    
    % Partial derivatives w.r.t stearing angle:
    df_da = [-v*T*sin(a)*cos(theta_mid) - (dD*v*T*cos(a)/(2*L))*sin(theta_mid);
             -v*T*sin(a)*sin(theta_mid) + (dD*v*T*cos(a)/(2*L))*cos(theta_mid);
             (v*T*cos(a))/L];
    % noise and errors from the control
    Au = [df_dv, df_da];


    sigma_v = 5; % mm/s uncertainty in speed
    sigma_a = 1*pi/180;  % rad uncertainty in steering angle
    % hardware/reading noise
    Cu = [sigma_v^2, 0; 
          0, sigma_a^2];


    % Use the law of error predictions, which gives the new uncertainty
    Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au';
    
    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
    % Plotting movement
    if PLOT_PATH
        plot(X,Y,'b.'); hold on; % plot path
        plot(CONTROL(1:kk-1,3), CONTROL(1:kk-1,4),'k.'); % plot path
        % Plot robot drivning Dead reckoning path
        plot_threewheeled([X(kk);Y(kk);A(kk)], 100, 612, 2, a, 150, 50, L); 
        % Plot robot drivning Ground Truth path
        plot_threewheeled([CONTROL(kk-1,3);CONTROL(kk-1,4);CONTROL(kk-1,5)], 100, 612, 2, a, 150, 50, L);
        drawnow();
    end
end

disp('Plotting ...');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure; 
    plot(X, Y, 'b.'); hold on; % By dead reconing 
    plot(CONTROL(:,3),CONTROL(:,4),'k.'); % Ground Truth
    title('Path taken by the robot [Wang]');
    xlabel('X [mm] World co-ordinates');
    ylabel('Y [mm] World co-ordinates');
    hold on;
        for kk = 1:5:N, % Change the step to plot less seldom, i.e. every 5th
            C = [P(kk,1:3);P(kk,4:6);P(kk,7:9)];
            plot_uncertainty([X(kk) Y(kk) A(kk)]', C, 1, 2);
        end;
    hold off;
    axis('equal');

% After the run, plot the results (X,Y,A), i.e. the estimated positions 
figure;
    subplot(3,1,1); plot(X, 'b'); title('X [mm]');
    subplot(3,1,2); plot(Y, 'b'); title('Y [mm]');
    subplot(3,1,3); plot(A*180/pi, 'b'); title('A [deg]');

% Plot the estimated variances (in X, Y and A) - 1 standard deviation
subplot(3,1,1); hold on;
    plot(X'+sqrt(P(:,1)), 'b:');
    plot(X'-sqrt(P(:,1)), 'b:');
hold off;
subplot(3,1,2); hold on;
    plot(Y'+sqrt(P(:,5)), 'b:');
    plot(Y'-sqrt(P(:,5)), 'b:');
hold off;
subplot(3,1,3); hold on;
    plot((A'+sqrt(P(:,9)))*180/pi, 'b:');
    plot((A'-sqrt(P(:,9)))*180/pi, 'b:');
hold off;


% Task 8: Compute Errors
Xgt = CONTROL(:,3);
Ygt = CONTROL(:,4);
Agt = CONTROL(:,5);

% error in estimation to ground truth
ex = X' - Xgt;
ey = Y' - Ygt;
ea = wrapToPi(A' - Agt);
% standard deviations of the estimated variance
sigma_x = sqrt(P(:,1));
sigma_y = sqrt(P(:,5));
sigma_a = sqrt(P(:,9));

% Plot Error vs Sigma
figure;
subplot(3,1,1);
plot(abs(ex),'k'); hold on;
plot(sigma_x,'r--'); title('|X error| vs sigma_x'); ylabel('mm'); legend('ex','sigma_x');

subplot(3,1,2);
plot(abs(ey),'k'); hold on;
plot(sigma_y,'r--'); title('|Y error| vs sigma_y'); ylabel('mm'); legend('ey','sigma_y');

subplot(3,1,3);
plot(abs(ea)*180/pi,'k'); hold on;
plot(sigma_a*180/pi,'r--'); title('|A error| vs sigma_a'); ylabel('deg'); xlabel('Time step');
legend('e','sigma');

%Verification Output
fprintf('Mean |X error| = %.2f mm, Mean x = %.2f mm\n', mean(abs(ex)), mean(sigma_x));
fprintf('Mean |Y error| = %.2f mm, Mean y = %.2f mm\n', mean(abs(ey)), mean(sigma_y));
fprintf('Mean |A error| = %.2f deg, Mean a = %.2f deg\n', ...
        mean(abs(ea))*180/pi, mean(sigma_a)*180/pi);


% Calculate Total Time
% N is the total number of samples, T is the sampling period (0.050s)
total_time = N * T; 

% Calculate Total Distance
% We sum the incremental displacements (dD) over the entire run.
% Note: Since dD was a local variable in the loop, we recalculate the sum here.
v_all = CONTROL(:,1);
a_all = CONTROL(:,2);
total_distance = sum(v_all .* cos(a_all) * T);

% Display the results in the command window
fprintf('\n--- Path Statistics ---\n');
fprintf('Total Time: %.2f seconds\n', total_time);
fprintf('Total Distance: %.2f mm (%.2f meters)\n', total_distance, total_distance/1000);

