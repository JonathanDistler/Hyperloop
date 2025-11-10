%Code that determines the angle with respect to the track of all corners of
%the pod given geometrical constraints and sensor data 


function [rqo, rpo, rmo, rno] = YawRoll(h1, h2, h3, h4)

    l1 = 10; % inches
    l2 = 8;  % inches

    delta_y12 = h1 - h2;
    delta_y43 = h4 - h3;

    delta_y14 = h1 - h4;
    delta_y23 = h2 - h3;

    theta_1 = atan(delta_y12 / l1);
    theta_2 = atan(delta_y43 / l1);
    theta = (theta_1 + theta_2) / 2;

    phi_1 = atan(delta_y14 / l2);
    phi_2 = atan(delta_y23 / l2);
    phi = (phi_1 + phi_2) / 2;

    % Rotation matrices
    M1 = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    M2 = [cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)];
    M_tot = M1 * M2;

    h_tot = 12; % inches
    d_tot = 8;  % inches

    h = h_tot / 2;
    d = d_tot / 2;

    r_qo = [d; 0; -h];
    r_po = [d; 0; h];
    r_mo = [-d; 0; h];
    r_no = [-d; 0; -h];

    % Transformed positions
    rqo = M_tot * r_qo;
    rpo = M_tot * r_po;
    rmo = M_tot * r_mo;
    rno = M_tot * r_no;
end

clear; clc;

N = 1000;
rqo_arr = zeros(3, N);
rpo_arr = zeros(3, N);
rmo_arr = zeros(3, N);
rno_arr = zeros(3, N);

for index = 1:N
    h1 = sin(index * 5);
    h2 = sin(index * 4.5);
    h3 = sin(index * 6);
    h4 = sin(index * 8);
    [rqo, rpo, rmo, rno] = YawRoll(h1, h2, h3, h4);

    rqo_arr(:, index) = rqo;
    rpo_arr(:, index) = rpo;
    rmo_arr(:, index) = rmo;
    rno_arr(:, index) = rno;
end

time = 1:N;

figure;
plot(time, rpo_arr(1,:), time, rpo_arr(2,:), time, rpo_arr(3,:));
title('Transformed R_{po} Components vs. Time');
xlabel('Time');
ylabel('R_{po}');
legend('X', 'Y', 'Z');

