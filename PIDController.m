function [output, e, integral] = PIDControl(R, sv, kp, ki, kd, prev_error, integral, dt)
    e = R - sv;                         % Error = setpoint - process variable
    integral = integral + e * dt;       % Integral term
    deriv = (e - prev_error) / dt;      % Derivative term (NOTE: fixed the formula)
    output = kp * e + ki * integral + kd * deriv;  % PID output
end

% Parameters
goal_distance = 0.5;  % 5 cm goal offset from track
pv = 0;                 % Initial position (process variable)
kp = 2; %Proportional gain
ki = 1; %Integral gain
kd = 0.5; % Derivative gain

prev_error = 0;         % Initialize previous error
integral = 0;           % Initialize integral term
dt = 0.001;            % Time step (0.1 ms)

% Data storage
time = [];
sv_history = [];
sp_history = [];
outputs = [];

% Simulation loop
for i = 1:1000
    [output, e, integral] = PIDControl(goal_distance, pv, kp, ki, kd, prev_error, integral, dt);
    
    % Update process variable (e.g., movement based on output)
    pv = pv + output * dt;

    % Store data
    time(end+1) = i * dt;
    sv_history(end+1) = pv;
    sp_history(end+1) = goal_distance;
    outputs(end+1) = output;

    % Update previous error
    prev_error = e;
end

% Plotting
plot(time, sv_history, 'b', time, sp_history, 'r--');
xlabel('Time (s)');
ylabel('Position (m)');
legend('State Variable (pv)', 'Setpoint (goal)');
title('PID Control Response');

