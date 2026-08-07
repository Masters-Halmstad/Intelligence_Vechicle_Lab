%
% Odometry with Khepera Mini Robot
%
% Ola Bengtsson, Björn Åstrand
% Version 5

% clear all;
% close all;
% The path plot can be toggled on and off to speed up the path plotting. 
% However, for debugging, you have the option to plot the path in the loop. 
% Otherwise, this is done after the loop is finished.
PLOT_PATH = 0; % [0, 1] 

% Use the following token to divide your code into different segments that
% is analogous to the following tasks:
% Task 1-4 = 1
% Task 5-6 = 0 
% (Note this is done so the examiner can quickly check your code) 
TASK = 1;
COMP_TERM = 1% [0, 1] If you use compensation term

% % Khepera settings 
% WHEEL_BASE = 53;                % [mm]
% WHEEL_DIAMETER = 15.3;          % [mm]

WHEEL_BASE = 45;                % [mm]
WHEEL_DIAMETER = 14;          % [mm]

PULSES_PER_REVOLUTION = 600;    %
MM_PER_PULSE = 15.3*pi/600 ;    % [mm / pulse]


% %%% Uncertainty settings, which are be the same for the left and right encoders
SIGMA_WHEEL_ENCODER = 0.5/12;   % The error in the encoder is 0.5mm / 12mm travelled
% Use the same uncertainty in both of the wheel encoders

SIGMAl = SIGMA_WHEEL_ENCODER;
SIGMAr = SIGMA_WHEEL_ENCODER;


% Uncertainty in Wheelbase (b)
% You typically need to define this sigma. Let's assume a small uncertainty 
% (e.g., 1mm standard deviation) if not strictly defined in variables.
SIGMA_b = 1.0; 

% Load encoder values
ENC = load('khepera_circle.txt');


% Transform encoder values (pulses) into distance travelled by the wheels (mm)
% Here you can change the sampling rate
Dr = ENC(1:100:end,2) * MM_PER_PULSE;
Dl = ENC(1:100:end,1) * MM_PER_PULSE;
N = max(size(Dr));

% Init Robot Position, i.e. (0, 0, 90*pi/180) and the Robots Uncertainty
X(1) = 0;
Y(1) = 0;
A(1) = 90*pi/180;
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

% Run until no more encoder values are available
disp('Calculating ...');
for kk=2:N,
    % Change of wheel displacements, i.e displacement of left and right wheels
    dDr = Dr(kk) - Dr(kk-1);
    dDl = Dl(kk) - Dl(kk-1);
    
    % Change of relative movements
    % dD = 0;
    % dA = 0.017;
    dD = (dDr + dDl)/2;   % You should write the correct one, which replaces the one above!
    dA = (dDr - dDl)/WHEEL_BASE;   % You should write the correct one, which replaces the one above!
    
    % Calculate the change in X and Y (World co-ordinates)
    % dX = 1;
    % dY = 1;
    dX = dD*cos(A(kk - 1)+dA/2);   % You should write the correct one, which replaces the one above!
    dY = dD*sin(A(kk - 1)+dA/2);   % You should write the correct one, which replaces the one above!

    if COMP_TERM
        % Write code here for using the compensation term
        % Wang Eq (3.3): Apply adjustment factor sin(x)/x
        if abs(dA) > 1e-6 % Avoid division by zero
             factor = sin(dA/2) / (dA/2);
              dX = dX * factor;
              dY = dY * factor;
        end
    end
    
    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = mod(A(kk-1) + dA, 2*pi);
    
    % Predict the new uncertainty in the state variables (Error prediction)
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];   % Uncertainty in state variables at time k-1 [3x3]    

    if TASK
        % Write the code for the odometry model used in Task 1-4 
        % Cu =   [1 0;0 1];               % Uncertainty in the input variables [2x2]
        % Axya = [1 0 0;0 1 0;0 0 1];     % Jacobian matrix w.r.t. X, Y and A [3x3]
        % Au =   [0 0;0 0;0 0];           % Jacobian matrix w.r.t. dD and dA [3x2]
        Axya = [1 0 -dD*sin(A(kk-1) + dA/2);
                0 1 dD*cos(A(kk-1) + dA/2);
                0 0 1]; % You should write the correct one, which replaces the one above!
        Au =[cos(A(kk-1) + dA/2) -dD*0.5*sin(A(kk-1)+dA/2);
             sin(A(kk-1) + dA/2) dD*0.5*cos(A(kk-1)+dA/2);
             0 1];   % You should write the correct one, which repleces the one above!
        CV = (SIGMAr^2 - SIGMAl^2)/(2*WHEEL_BASE);
        Cu = [(SIGMAr^2 + SIGMAl^2)/ 4 CV; CV (SIGMAr^2 + SIGMAl^2)/(WHEEL_BASE^2)];   % You should write the correct one, which replaces the one above!
        
        % Use the law of error predictions, which gives the new uncertainty
        Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au';
    else
        % Write the code for the odometry model used in Task 5-6 
        % Cu =   [1 0;0 1];               % Uncertainty in the input variables [2x2]
        % Axya = [1 0 0;0 1 0;0 0 1];     % Jacobian matrix w.r.t. X, Y and A [3x3]
        % Au =   [0 0;0 0;0 0];           % Jacobian matrix w.r.t. dD and dA [3x2]

        Axya = [1 0 -dD*sin(A(kk-1) + dA/2);
                0 1 dD*cos(A(kk-1) + dA/2);
                0 0 1]; % You should write the correct one, which replaces the one above!

        half_theta = A(kk-1) + dA/2;
        term_cos = 0.5 * cos(half_theta);
        term_sin = 0.5 * sin(half_theta);
        term_d_b = dD / (2 * WHEEL_BASE)

        Au = [ (term_cos - term_d_b * sin(half_theta)), (term_cos + term_d_b * sin(half_theta));
               (term_sin + term_d_b * cos(half_theta)), (term_sin - term_d_b * cos(half_theta));
               (1 / WHEEL_BASE),                        (-1 / WHEEL_BASE) ];   % You should write the correct one, which repleces the one above!

        Cu = [SIGMAr^2, 0; 
                0, SIGMAl^2];   % You should write the correct one, which replaces the one above!

        % Jacobian J_b (Ab) - w.r.t Wheelbase (b)
        % Represents how uncertainty in the wheelbase measurement affects Pose.
        % Derived from partial derivatives of Eq (8) w.r.t 'b'.
        Cb = SIGMA_b^2;

        Ab = [  dD * (dA / (2 * WHEEL_BASE)) * sin(half_theta);
               -dD * (dA / (2 * WHEEL_BASE)) * cos(half_theta);
               -dA / WHEEL_BASE ];


        % Use the law of error predictions, which gives the new uncertainty
        Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au' + Ab * Cb * Ab';
    end

    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
    if PLOT_PATH, % 
        % Plotting movement
        plot(X,Y,'k.'); % plot path
        plot_khepera([X(kk);Y(kk);A(kk)], WHEEL_DIAMETER, WHEEL_BASE, 3);
        drawnow();
    end
end;


disp('Plotting ...');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure; 
    %plot(X, Y, 'b.');
    title('Path taken by the robot [Wang]');
    xlabel('X [mm] World co-ordinates');
    ylabel('Y [mm] World co-ordinates');
    hold on;
        for kk = 1:1:N,
            C = [P(kk,1:3);P(kk,4:6);P(kk,7:9)];
            plot_uncertainty([X(kk) Y(kk) A(kk)]', C, 1, 2, 'k');
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


%-------------------------------------------------------
% Comparison code for task 3 with different sampling rate
%---------------------------------------------------------

% X2 = X; Y2 = Y; A2 = A;
% X5 = X; Y5 = Y; A5 = A;
% X10 = X; Y10 = Y; A10 = A;
% 
% figure;
% 
% Subplot 1: Trajectory (X and Y)
% subplot(2,1,1);
% plot(X2, Y2, 'k', 'LineWidth', 1); hold on;
% plot(X5, Y5, 'b--', 'LineWidth', 1);
% plot(X10, Y10, 'r:', 'LineWidth', 1.5);
% legend('Step 2', 'Step 5', 'Step 10');
% title('Trajectory Comparison: (x, y) State Variables');
% xlabel('X [mm]'); ylabel('Y [mm]');
% axis equal; grid on;
% 
% % Subplot 2: Heading Angle (A)
% subplot(2,1,2);
% plot(A2*180/pi, 'k', 'LineWidth', 1); hold on;
% plot(A5*180/pi, 'b--', 'LineWidth', 1);
% plot(A10*180/pi, 'r:', 'LineWidth', 1.5);
% title('Heading Comparison: \Theta State Variable');
% xlabel('Sample Index');
% ylabel('Degrees');
% grid on;


% % Matlab comparision code
% if COMP_TERM
% if abs(dA) > 1e-6 % Avoid division by zero
%     factor = sin(dA/2) / (dA/2);
%     dX = dX * factor;
%     dY = dY * factor;
% end
% end
% 
% % Code to compare the models 
% % Value of X, Y , A and P with no componsation:
% X_no_comp = X;
% Y_no_comp = Y;
% A_no_comp = A;
% P_no_comp = P;
% 
% % Value of X, Y , A and P with componsation:
% X_with_comp = X;
% Y_with_comp = Y;
% A_with_comp = A;
% P_with_comp = P;
% 
% >> figure;
% % Plot Trajectories
% plot(X_with_comp, Y_with_comp, 'b-', 'LineWidth', 2); hold on;
% plot(X_no_comp, Y_no_comp, 'r--', 'LineWidth', 1.5);
% 
% % Plot the final uncertainty ellipses for both
% 
% plot_uncertainty([X_with_comp(end); Y_with_comp(end); 
%                     A_with_comp(end)], P_with_comp, 1, 2); 
% plot_uncertainty([X_no_comp(end); Y_no_comp(end);
%                     A_no_comp(end)], P_no_comp, 1, 3); 
% 
% legend('With Compensation', 'No Compensation', 
% 'Uncertainty (With)', 'Uncertainty (No)');
% title('Comparison of State and Covariance (Uncertainty)');
% axis equal; grid on;
% 
% % Code to Calculate the Euclidean distance between the two final points
%     final_diff = sqrt((X_no_comp(end) - X_with_comp(end))^2 
%                     + (Y_no_comp(end) - Y_with_comp(end))^2);
% 
%     fprintf('The difference between models at the end of
%                     the path is: %.6f mm\n', final_diff);