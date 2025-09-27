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
%Angular velocity (rad/s) rate of change (rad/s^2)
ang_vel_diff = diff(ang_vel);    

%first order line of best fit between time and linear velocity
coefficients = polyfit(elapsed_time, lin_vel, 1); % Change in angular velocity
slope=coefficients(1)

%%%%%%%%%%%%%%%%%Setup for MoI Calculations
Ri=3*1/2; %Radius of inner hole in inches
Ro=12+Ri; %Radius of outer radius including hole, in inches
Ri_m=convlength(Ri,"in","m");
Ro_m=convlength(Ro,"in","m");
Depth=convlength(.3,"in","m");
V=pi*(Ro_m^2-Ri_m^2)*Depth;
Rho=2699; %Kg/m^3, density of Aluminum
M=V*Rho; %Mass in Kg, determined from material and dimensions

MoI=calculate(M,Ro_m,Ri_m);
function MoI = calculate(M,Ro,Ri)
    
    MoI_Outer=1/2*(M)*Ro^2;
    MoI_Inner=-1/2*(M)*Ri^2;
    MoI=MoI_Outer-MoI_Inner;
end

%I*Theta-dd=T
Theta_dd=mean(ang_vel_diff); %average angular acceleration
I=MoI;
T=Theta_dd*I; %Torque in Nm
