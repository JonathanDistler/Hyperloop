%Code that determines the angle with respect to the track of all corners of
%the pod given geometrical constraints and sensor data 

function [rqo, rpo, rmo, rno] = YawRoll(h1, h2, h3, h4)

    l1 = 10; % inches
    l2 = 8;  % inches

    delta_y12 = h1 - h2; %height differences 
    delta_y43 = h4 - h3; %height differences

    delta_y14 = h1 - h4; %height differences
    delta_y23 = h2 - h3; %height differences

    theta_1 = atan(delta_y12 / l1); %theta averaging methodology for center of bounding box
    theta_2 = atan(delta_y43 / l1);
    theta = (theta_1 + theta_2) / 2;

    phi_1 = atan(delta_y14 / l2); %phi averaging methodology for center of bounding box
    phi_2 = atan(delta_y23 / l2);
    phi = (phi_1 + phi_2) / 2;

    % Rotation matrices
    M1 = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; %about z axis (yaw)
    M2 = [cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)]; %about y axis (pitch)
    M_tot = M1 * M2;

    h_tot = 12; % inches, the bounding box length
    d_tot = 8;  % inches, the bounding box diameter

    %defining the origin as the center of the bounding box, thus everything
    %is relative to half the length or diamter from center of mass 
    h = h_tot / 2;
    d = d_tot / 2;

    %defining basic vectors with respect to the center of the bounding box 
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

%sets up arrays for graphing, all have 3 dimensions
N = 1000;
rqo_arr = zeros(3, N);
rpo_arr = zeros(3, N);
rmo_arr = zeros(3, N);
rno_arr = zeros(3, N);

%iterating through N time steps
for index = 1:N
    %defining a time dependent distance with the time of flight sensors 
    h1 = sin(index * 5);
    h2 = sin(index * 4.5);
    h3 = sin(index * 6);
    h4 = sin(index * 8);

    %calling function and getting the positional readouts
    [rqo, rpo, rmo, rno] = YawRoll(h1, h2, h3, h4);

    %adding to the new arrays
    rqo_arr(:, index) = rqo;
    rpo_arr(:, index) = rpo;
    rmo_arr(:, index) = rmo;
    rno_arr(:, index) = rno;
end

%COMMENT OUT IF YOU DON'T WANT TO SAVE THE FIGURES
save_val=true; 

%defines time as the length of the number of steps
time = 1:N;

%plotting; time vs the components of R_po
figure;
plot(time, rpo_arr(1,:), time, rpo_arr(2,:), time, rpo_arr(3,:));
title('Transformed R_{po} Components vs. Time');
xlabel('Time');
ylabel('R_{po}');
legend('X', 'Y', 'Z');

if (save_val)
    saveas(gcf, 'R_PO.png');
end

%plotting; time vs the components of R_qo
figure;
plot(time, rqo_arr(1,:), time, rqo_arr(2,:), time, rqo_arr(3,:));
title('Transformed R_{qo} Components vs. Time');
xlabel('Time');
ylabel('R_{qo}');
legend('X', 'Y', 'Z');

if (save_val)
    saveas(gcf, 'R_QO.png');
end


%plotting; time vs the components of R_mo
figure;
plot(time, rmo_arr(1,:), time, rmo_arr(2,:), time, rmo_arr(3,:));
title('Transformed R_{mo} Components vs. Time');
xlabel('Time');
ylabel('R_{mo}');
legend('X', 'Y', 'Z');

if (save_val)
    saveas(gcf, 'R_MO.png');
end

%plotting; time vs the components of R_no
figure;
plot(time, rno_arr(1,:), time, rno_arr(2,:), time, rno_arr(3,:));
title('Transformed R_{no} Components vs. Time');
xlabel('Time');
ylabel('R_{no}');
legend('X', 'Y', 'Z');

if (save_val)
    saveas(gcf, 'R_NO.png');
end
