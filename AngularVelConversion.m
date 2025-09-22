filename = "/MATLAB Drive/MagneticBrakingFlywheel.csv";  % File location in MATLAB Online

data = readtable(filename);

lin_vel = data{2:end,5}; %linear velocity from 2nd row on 
elapsed_time = data{2:end, 1}; %elapsed time from 2nd row on 


% Linear velocity (cm/s) rate of change
lin_vel_diff = diff(lin_vel);           % Change in linear velocity

% Radius in cm
radius = 14;    

% Angular velocity (rad/s)
ang_vel = lin_vel ./ radius; 
%Angular velocity (rad/s) rate of change
ang_vel_diff = diff(ang_vel);    

%first order line of best fit between time and linear velocity
coefficients = polyfit(elapsed_time, lin_vel, 1); % Change in angular velocity
slope=coefficients(1)

M=5; %mass in kg, measured anecodtally [NEED ACTUAL VALUE]
MoI=(1/2)*M*radius^2;

