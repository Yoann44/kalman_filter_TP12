% Main code for the EKF implementation in the Robotics Practivals course
clear all;clc; close all;

plot_enable = true;
cut_gps_enable = false;
t1_cut = 100;
t2_cut = 400;

%% Constants
g = 9.81;        % gravity [m/s^2] (could come from a gravity model)

% Values required to calculate initial value for k in Baro observation
% equation:
M = 0.02897;  % Air molar mass [kg/mol]
R = 8.3144598;  % Universal gas constant [J/kg/K]
T = 298.15;  % Air temperature (carefull about the units) [K]

%% Loading observations
load('Quad_test_data.mat')
gps_measurements  = observations.gps.z;
time_gps          = observations.gps.time;

baro_measurements = observations.baro.z;
time_baro         = observations.baro.time;

ground_truth      = observations.gps.z;
time_ground_truth = observations.gps.time;

if cut_gps_enable
    t1_cut = min(time_gps) + t1_cut;
    t2_cut = min(time_gps) + t2_cut;
    index = (time_gps < t1_cut | time_gps > t2_cut);
    gps_measurements = gps_measurements(index);
    time_gps = time_gps(index);
end

%% KALMAN filter uncertainty parameters
%Standard deviation of the GPS position white noise
sigma_z_gps  = 5;       % [m]

%Standard deviation of the BARO readings white noise
sigma_z_baro = 5;       % [Pa]

% Process model noise
sigma_a_dot  = 1;           % [m/s^2 /s]
sigma_p0_dot = 0.01;          % [Pa /s]
sigma_k_dot  = 1e-9;         % [s^2/m^2 /s]

%% Generate the corresponding coviariance matrices
%% INITIAL VALUES
%States
X_0 = [observations.gps.z(1); 0; 0; observations.gps.z(1); M/(R*T); observations.baro.z(1)];

%Covariances
P_0 = diag([5.0; 0.5; 0.5; 5.0; 1e-6; 5.0].^2);

% Initialize the A PRIORI estimate
x_tilde = X_0;
P_tilde = P_0;


% State vector initial A POSTERIORI covariance matrix P_0
x_hat = x_tilde;
P_hat = P_tilde;

% Observations covariance matrices
R_gps  = [sigma_z_gps.^2];
R_baro = [sigma_z_baro.^2];

% Calculation of Kalman filter matrices
% Process model noise [PSD]
Q = diag([sigma_a_dot, sigma_k_dot, sigma_p0_dot].^2);

% Dynamic matrix
F = [
    0   1   0   0   0   0
    0   0   1   0   0   0
    0   0   0   0   0   0
    0   0   0   0   0   0
    0   0   0   0   0   0
    0   0   0   0   0   0
    ];

% Noise shaping matrix
G = [
    0   0   0
    0   0   0
    1   0   0
    0   0   0
    0   1   0
    0   0   1
    ];

% MISC
i = 1;
j = 1;
k = 1;
time_last = min(time_baro(i),time_gps(j));

%% EKF Solution computation
% Preallocation of variables
kk     = length(union(time_gps,time_baro));
X      = nan(6,kk);
P      = nan(6,6,kk);
P_diag = nan(6,kk); % Only diagonal elements of covariance matrix
t      = nan(1,kk);

while true
    try
        X(:,k) = x_hat;
        P(:,:,k) = P_hat;
        P_diag(:,k) = diag(P_hat);
        
        if time_baro(i)<time_gps(j)
            % Kalman filter prediction step:
            [time_last,x_tilde,P_tilde] = prediction(time_baro(i),time_last,x_hat,P_hat,Q,F,G);
            
            % Kalman filter update step for Baro measurements:
            [x_hat,P_hat] = BARO_update_const_a(baro_measurements(i),x_tilde,P_tilde,R_baro,g);
            t(k) = time_baro(i);
            i = i + 1;
        else
            % Kalman filter prediction step:
            [time_last,x_tilde,P_tilde] = prediction(time_gps(j),time_last,x_hat,P_hat,Q,F,G);
            
            % Kalman filter update step for GPS measurements:
            [x_hat,P_hat] = GPS_update_const_a(gps_measurements(j),x_tilde,P_tilde,R_gps);
            t(k) = time_gps(j);
            j = j + 1;
        end
        
        k = k+1;
    
    catch
        disp(k-1)
        break
    end
    
end

% Removing non-allocated values
X = X(:,1:k-1);
P = P(:,:,1:k-1);
P_diag = P_diag(:,1:k-1);
t = t(:,1:k-1);

%% Plot output
set(groot,'DefaultAxesFontSize',18);
set(groot,'DefaultLineLineWidth',1.5);

if plot_enable
    % Use a time offset in order to have mindfull scale
    time_offset = min([min(t) min(time_gps) min(time_baro)]);
    
    % GPS vs Filtered
    plot(t - time_offset, X(1,:), time_gps - time_offset, gps_measurements)
    xlabel("Time [s]");
    ylabel("Altitude [m]")
    legend("Filtered altitude", "Raw GPS altitude");
    
    [t_aligned, a1, a2] = timealign(time_gps, t, gps_measurements, X(1,:)', 1);

    figure;
    plot(t_aligned - time_offset, abs(a1-a2));
    xlabel("Time [s]");
    ylabel("Difference [m]")
    legend("Difference filtered/raw");
    
    % Filtered wo cuts vs filtered with cuts
    if cut_gps_enable
        load('X_ref.mat')

        figure;
        plot(t_ref - time_offset, X_ref(1,:), t - time_offset, X(1,:))
        xlabel("Time [s]");
        ylabel("Altitude [m]")
        legend("Filtered altitude no GPS cuts", "Filtered altitude with GPS cuts");

        [t_aligned, a1, a2] = timealign(t_ref, t, X_ref(1,:)', X(1,:)', 1);

        figure;
        plot(t_aligned - time_offset, a1-a2);
        xlabel("Time [s]");
        ylabel("Difference [m]")
        legend("abs(filt w gps - filt wo GPS)");
        
        err_std = std(a1 - a2);
        fprintf("Standard deviation : %f\n", err_std);
    end

    figure;
    subplot(3,1,1);
    plot(t - time_offset, X(4,:));
    legend("Takeoff altitude");
    xlabel("Time [s]");
    ylabel("Altitude [m]");

    subplot(3,1,2);
    plot(t - time_offset, X(5,:))
    legend("K = M/RT");
    xlabel("Time [s]");
    ylabel("[kg^2/(J*mol)]");

    subplot(3,1,3);
    plot(t - time_offset, X(6,:))
    legend("Takeoff pressure");
    xlabel("Time [s]");
    ylabel("Pressue [pa]");
end
