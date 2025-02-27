function [] = ekf_localization()
 
% Homework for ekf localization
% Modified by YH on 09/09/2019, thanks to the original open source
% Any questions please contact: zjuyinhuan@gmail.com

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
    convQ = noiseQ.^2;

    % Covariance Matrix for observation
    convR = noiseR.^2;

    % Other Intial
    sigma_EKF = eye(3);
    miu_EKF = [0 0 0]';

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        miu_EKF_estimate = doMotion(miu_EKF, u);
        jF = jacobF(miu_EKF, u);
        sigma_estimate = jF * sigma_EKF * jF' + convQ;

        % Update Step
        zPred = doObservation(z, miu_EKF_estimate);
        jH = jacobH(miu_EKF_estimate);
        K = sigma_estimate * jH' / (jH * sigma_estimate * jH' + convR);
        miu_EKF = miu_EKF_estimate + K * (z - zPred);
        sigma_EKF = (eye(3) - K * jH) * sigma_estimate;


        % Update
        xEkf = miu_EKF;
        % -----------------------------

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    v = sqrt(u(1)^2 + u(2)^2);
    x(1) = x(1) - v/u(3)*sin(x(3)) + v/u(3)*sin(x(3) + u(3)*dt);
    x(2) = x(2) + v/u(3)*cos(x(3)) - v/u(3)*cos(x(3) + u(3)*dt);
    x(3) = x(3) + u(3)*dt;
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    v = sqrt(u(1)^2 + u(2)^2);
    jF = [1 0 -v/u(3)*cos(x(3)) + v/u(3)*cos(x(3) + u(3)*dt);
          0 1 -v/u(3)*sin(x(3)) + v/u(3)*sin(x(3) + u(3)*dt);
          0 0 1];
end

%Observation Model
function x = doObservation(z, xPred)
    x = xPred;
 end

%Jacobian of Observation Model
function jH = jacobH(x)
    jH = eye(3);
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % 绘制 Odemetry 和 EKF 的距离误差
    figure;

    % 计算每个时间点的距离误差
    odemetry_distance_error = sqrt((estimation.xTruth(:,1) - estimation.xOdom(:,1)).^2 + (estimation.xTruth(:,2) - estimation.xOdom(:,2)).^2);
    EKF_distance_error = sqrt((estimation.xTruth(:,1) - estimation.xEkf(:,1)).^2 + (estimation.xTruth(:,2) - estimation.xEkf(:,2)).^2);
    
    disp('odemetry_distance_error:')
    disp(sum(odemetry_distance_error))
    disp('EKF_distance_error:')
    disp(sum(EKF_distance_error))

    % 绘制距离误差随时间的变化
    subplot(2, 1, 1);
    plot(odemetry_distance_error, '-r', 'DisplayName', 'Odemetry Distance Error');
    hold on;
    plot(EKF_distance_error, '-b', 'DisplayName', 'EKF Distance Error');
    xlabel('Time Step');
    ylabel('Distance Error');
    title('Distance Error Over Time');
    legend;
    hold off;

    % 计算每个时间点的角度误差
    odemetry_angle_error = abs(estimation.xTruth(:,3) - estimation.xOdom(:,3));
    EKF_angle_error = abs(estimation.xTruth(:,3) - estimation.xEkf(:,3));

    % 绘制角度误差随时间的变化
    subplot(2, 1, 2);
    plot(odemetry_angle_error, '-r', 'DisplayName', 'Odemetry Angle Error');
    hold on;
    plot(EKF_angle_error, '-b', 'DisplayName', 'EKF Angle Error');
    xlabel('Time Step');
    ylabel('Angle Error');
    title('Angle Error Over Time');
    legend;
    hold off;

    disp('odemetry_angle_error:')
    disp(sum(odemetry_angle_error))
    disp('EKF_angle_error:')
    disp(sum(EKF_angle_error))

end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end