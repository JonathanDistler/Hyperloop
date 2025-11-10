%Code that determines the angle with respect to the track of all corners of
%the pod given geometrical constraints and sensor data 

l1=10; %[inches] long side of the measurement platform (connects 1-2 and 4-3)
l2=8; %[inches] short side of the measurement platform (connects 1-4 and 2-3)

%sensor order is from front left to front right counter-clockwise


%Readouts are place-holders at the moment - ideally they would be live time
%readouts from 4 time of flight sensors 

h1=5; %readout from the first sensor
h2=6; %readout from the second sensor
h3=4; %readout from the third sensor
h4=7; %readout from the fourth sensor 

delta_y12=h1-h2; %height differences between 1 and 2
delta_y43=h4-h3; %height differences between 4 and 3


delta_y14=h1-h4; %height differences between 1 and 4
delta_y23=h2-h3; %height differences between 2 and 3

theta_1=atan(delta_y12/l1);
theta_2=atan(delat_y43/l1); 

theta=(theta_1+theta_2)/2; %averaged theta value for the center of the system

phi_1=atan(delta_y14/l2);
phi_2=atan(delta_y23/l2);

phi=(phi_1+phi_2)/2; %averaged phi value for the center of the system



M1=[cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; %matrix one representing yaw
M2=[cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)]; %matrix two representing roll

M_tot=M1*M2; 
%defined a rectangle with points on each corner to define the cross section
%of the aeroshell

h_tot=12; %inches
d_tot=8; %inches

h=h_tot/2; 
d=d_tot/2; 

r_qo=[d; 0; -h];
r_po=[d; 0; h];
r_mo=[-d; 0; h];
r_no=[-d; 0; -h];

%Multiplying all of the positional vectors by the matrix to determine new
%components for a given time 

r_qo_prime=M_tot*r_qo 
r_po_prime=M_tot*r_po
r_mo_prime=M_tot*r_mo
r_no_prime=M_tot*r_no







