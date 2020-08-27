%% Simulation: This is the simulation second draft. Physics engine will run inside this

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Version 1_2 is the simulation restructured   %%%%%%%%%
%%%%%%%%% to use a version of the freerotate method    %%%%%%%%%
%%%%%%%%% which tracks and updates object.position     %%%%%%%%%
%%%%%%%%% by using orientation and the previous        %%%%%%%%%
%%%%%%%%% object1.position, instead of absolute angles %%%%%%%%%
%%%%%%%%% and the initial value of object1.position.   %%%%%%%%%
%%%%%%%%% This approach will hopefully create a smooth %%%%%%%%%
%%%%%%%%% transition to Version 2, where forces are    %%%%%%%%%
%%%%%%%%% applied and EOMs are solved.                 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

% Define frames: need to make these globals
global Nframe
global dt
global T

%define simulation space frame
Nframe = [1 0 0; 0 1 0; 0 0 1];

T = 3;                      % duration of simulation
dt = 0.01;
t = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% STUFF THAT WILL HAPPEN IN THE GUI %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define initial conditions
phi_0 = 0;  %initial spin orientation
psi_0 = 0;  %initial precession orientation
theta_0 = 20;  % The newtation angle, it is between the spin axis and the angular momentum vector
phi_dot = 10; % rotational rate in radians per second
psi_dot = 0; %rad/sec
theta_dot = 0;
%define object and load initial conditions into its properties
object1 = Cylinder;
object1.mass = 1; object1.radius = 4; object1.height = 20;
object1.orientation = [phi_0; psi_0; theta_0];
N = 20;  %face resolution of the cylinder defined below
[X,Y,Z] = cylinder([object1.radius object1.radius object1.radius],N);
%define point cloud of the cylinder, this is technically the Bframe point
%cloud since we have not rotated the cylinder into the Nframe yet, but if
%there are no rotations to be done, points should equal object1.position at
%the end of this section :)
object1.pointcloud = pointcloud(X,Y,Z*object1.height); object1.pointcloud(3,:) = object1.pointcloud(3,:) - object1.height/2;

%define initial rotation of the object, if any attitude exists
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
object1.position = R_nb*object1.pointcloud;
disp(object1.position(:,N-1));    %displaying the initial tracked point of the point cloud for troubleshooting purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% BEHIND THE SCENES %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Conversions of spin rates and entered into class phelp roperties
phi_dot = (180/pi)*phi_dot; %deg per second
% psi_dot = 0; %rad/sec
psi_dot = (180/pi)*psi_dot; %deg per sec
theta_dot = (180/pi)*theta_dot; %deg per sec
object1.spin_rates = [phi_dot; psi_dot; theta_dot];

%% Loop
%this for loop will eventually be "while Applied_Force or 
%Applied_Moment == 0" and will live inside another for loop called "while sim = true"
%which can perhaps be canceled from the GUI with an off switch or something
Position = zeros(3,(N+1)*3,T*100+1);        %preallocate position vector as a function of the total simulation time
n = 1;
Position(:,:,n) = object1.position;   %and define initial position
fprintf('Runtime for a simulation length of %f seconds\n',T)
tic
while t <= T        %want this to eventually be "While objectX exists, obey the laws of physics", which will be a loop within while(sim == true) do t = t + dt
    %%% ADD THE BELOW IN FOR A NICE MOVIE, BUT PROGRAM RUNTIME WILL INCREASE %%%
    % THE MOVIE RUNS AS THE PROGRAM IS RUNNING %
    plot3(Position(1,:,n),Position(2,:,n),Position(3,:,n),'b.'); axis equal; hold on;
    plot3(Position(1,N-1,n),Position(2,N-1,n),Position(3,N-1,n),'r-o'); hold off
    text(0,object1.height,sprintf('Timestamp: %f',t))
    M(n) = getframe(gcf); 
    
    t = t + dt;     %define the time stamp at which we will calculate
    n = n + 1;
    %disp('time incremented')
    [object1.orientation, object1.position] = object1.freerotate(dt,Nframe);
    Position(:,:,n) = object1.position;

end
toc
disp('Final position of tracked point');
disp(object1.position(:,N-1));
