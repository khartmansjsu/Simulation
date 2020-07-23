function [object, position] = freerotate(object, position)

%resources
% http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html#mom
% http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node67.html

%define object properties at this time stamp
r = object.radius;      % radius of the cylinder (need this to eventually be adjustable given forces introduced)
% m = object.mass;      % mass of the cylinder (need this to eventually be adjustable given forces introduced)
h = object.height;      % height of the cylinder (need this to eventually be adjustable given forces introduced)
phi = object1.orientation(1);
psi = object1.orientation(2);
theta = object1.orientation(3);

phi = phi + phi_dot*dt;   % angles grow, they do not reset after 360
psi = psi + psi_dot*dt;   % angles grow, they do not reset after 360

R_na = [cosd(psi) -sind(psi) 0;        %Precession rotation matrix
    sind(psi) cosd(psi) 0;
    0 0 1];
R_ag = [1 0 0;                         %Newtation rotation matrix
        0 cosd(theta) -sind(theta);     
        0 sind(theta) cosd(theta)];
R_gb = [cosd(phi) -sind(phi) 0;        %Spin rotation matrix
        sind(phi) cosd(phi) 0;
        0 0 1];
 R_nb = R_na*R_ag*R_gb;                %Matrix from B-->N
 Bframe = Nframe*R_nb;                 %Define B frame
 B_x = Bframe(:,1);                    %Extract B unit vectors
 B_y = Bframe(:,2);
 B_z = Bframe(:,3);
% w = psi_dot*N_x + phi_dot*B_x;
% I = [(m/12)*(3*r^2+h^2) 0 0;                 %Check if mass properties matrix is constant
% 0 (m/12)*(3*r^2+h^2) 0;
% 0 0 (m*r^2)/2];                              %constant **
position(:,(t*1/dt)+1) = (r/2)*B_x + (r/4)*B_y + (h/2)*B_z;  %redefine position of point being tracked
end