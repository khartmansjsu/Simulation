%% Simulation: This is the simulation first draft. Physics engine will run inside this

% Define frames: need to make these globals

clc
clear all
%close all

global Nframe
global dt
global T

%define simulation space frame
Nframe = [1 0 0; 0 1 0; 0 0 1];

T = 7.25;                      %duration of simulation
dt = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% STUFF THAT WILL HAPPEN IN THE GUI %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define initial conditions
t = 0;
phi_0 = 0;  %initial spin orientation
psi_0 = 0;  %initial precession orientation
theta = 20;  % The newtation angle, it is between the spin axis and the angular momentum vector
phi_dot = 5; % rotational rate in radians per second
psi_dot = 10.66; %rad/sec
theta_dot = 0;
%define object and load initial conditions into its properties
object1 = Cylinder;
object1.mass = 1; object1.radius = 4; object1.height = 20;
object1.orientation = [phi_0; psi_0; theta];
%define point on the object we want to track as a vector from the origin to
%that point
          R_na = [cosd(object1.orientation(2)) -sind(object1.orientation(2)) 0;        %Precession rotation matrix
              sind(object1.orientation(2)) cosd(object1.orientation(2)) 0;
              0 0 1];
          R_ag = [1 0 0;                                                       %Newtation rotation matrix
                  0 cosd(object1.orientation(3)) -sind(object1.orientation(3));     
                  0 sind(object1.orientation(3)) cosd(object1.orientation(3))];
          R_gb = [cosd(object1.orientation(1)) -sind(object1.orientation(1)) 0;        %Spin rotation matrix
                  sind(object1.orientation(1)) cosd(object1.orientation(1)) 0;
                  0 0 1];
           R_nb = R_na*R_ag*R_gb;                %Matrix from N-->B
object1.position = R_nb*[object1.radius/2; object1.radius/4; object1.height];
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% BEHIND THE SCENES %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Conversions of spin rates and entered into class phelp roperties
phi_dot = (180/pi)*phi_dot; %deg per second
% psi_dot = 0; %rad/sec
psi_dot = (180/pi)*psi_dot; %deg per sec
object1.spin_rates = [phi_dot; psi_dot; theta_dot];

%% Loop
%this for loop will eventually be "while Applied_Force or 
%Applied_Moment == 0" and will live inside another for loop called "while sim = true"
%which can perhaps be canceled from the GUI with an off switch or something
Position = zeros(3,T*100+1);        %preallocate position vector as a function of the total simulation time
Position(:,t*100+1) = object1.position;   %and define initial position
n = 1;
tic
while t <= T  %start at 2 for now, but eventually want it to be t_of_object_initialization + dt
    t = t + dt;     %define the time stamp at which we will calculate
    n = n + 1;
    %disp('time incremented')
    [object1.orientation, Position(:,n)] = object1.freerotate(dt,Nframe);
    %disp('object rotated')
    %disp('position plot updated')
end
toc

figure;
plot3(Position(1,:),Position(2,:),Position(3,:),'b')
axis equal
xlabel('x'); ylabel('y'); zlabel('z')
zlim([-1 object1.height+2]);
title('Position of a point of the top face of a directly precessing cylinder')