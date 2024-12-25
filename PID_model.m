% Define Parameters
setpoint = 200; % Target displacement in mm
kp = 0.3; % Proportional gain
ki = 0.01; % Integral gain
kd = 0.1; % Derivative gain

thetaBeamFactor = 1/7; % Conversion from servo angle to beam angle
g = 9.81; % Acceleration due to gravity in m/s^2
dt = 0.01; % Time step for simulation (s)
simulationTime =5; % Total simulation time (s)

servoMin = -1.22; % Minimum servo angle in radians
servoMax = 1.22;  % Maximum servo angle in radians

% Initialize Variables
thetaServo = 0; % Servo angle in radians
thetaBeam = 0; % Beam angle in radians
acceleration = 0; % Ball acceleration (m/s^2)
velocity = 0; % Ball velocity (m/s)
displacement = 0.4; % Ball displacement (m)

integralError = 0; % For PID integral term
previousError = 0; % For PID derivative term

% Time vector for simulation
time = 0:dt:simulationTime;
displacementHistory = zeros(size(time)); % Store displacement for plotting
servoHistory = zeros(size(time)) ;

% Simulation Loop
for i = 1:length(time)
    % Calculate error
    error = setpoint - displacement * 1000; % Convert displacement to mm

    % PID Controller
    derivativeError = (error - previousError) / dt; % Derivative term
    
    % Update integral term only if within servo limits
    if servoMin <= thetaServo && thetaServo <= servoMax
        integralError = integralError + error * dt; % Integral term
    end
    
    % PID output (Servo angle)
    thetaServo = kp * error + ki * integralError + kd * derivativeError; 
    
    % Apply servo angle limits
    thetaServo = max(servoMin, min(servoMax, thetaServo)); % Clamp to limits
    
    servoHistory(i) = thetaServo * (180/pi) ; % convert it to degrees 
    % Update previous error
    previousError = error;

    % System Dynamics
    thetaBeam = thetaServo * thetaBeamFactor; % Beam angle
    acceleration = g * sin(thetaBeam); % Acceleration of the ball
    velocity = velocity + acceleration * dt; % Update velocity
    displacement = displacement + velocity * dt; % Update displacement

    % Store displacement for plotting
    displacementHistory(i) = displacement * 100; % Convert to cm for output
end

% Plot Results
subplot(2, 1, 1);
plot(time, displacementHistory, 'b', 'LineWidth', 1.5);
title('Ball Displacement Over Time');
xlabel('Time (s)');
ylabel('Displacement (cm)');
grid on;

subplot(2, 1, 2);
plot(time, servoHistory, 'r', 'LineWidth', 1.5);
title('Servo Angle Over Time');
xlabel('Time (s)');
ylabel('Servo Angle (radians)');
grid on;
