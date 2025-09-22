clear all
close all
clc

% Parameters
M1 = 81;        % mass of pod [kg]
g = 9.81;       % gravity [m/s^2]
u1 = 0.1;       % static friction coefficient
PodForce = (M1/2)*g*u1;
fos = 1.5;      % safety factor
lbfconversion = 0.7375621493; % Nm -> ft*lbf

% Values
nVals = 2:2:10;                 % number of wheels
RadiiVals = linspace(0.05,0.25,5); % wheel radii [m]

% Create grid for 3D plot
[N, R] = meshgrid(nVals, RadiiVals);  % N: wheel counts, R: radii

% Compute torque with safety factor
Torque = (PodForce ./ N) .* R * fos * lbfconversion;

% 3D surface plot
figure;
surf(N, R, Torque)
xlabel('Number of wheels');
ylabel('Wheel radius [m]');
zlabel('Torque with tolerance [Ft*lbf]');
title('Torque vs. Wheel Radius and Number of Wheels');
colorbar
grid on
view(35,45);
