%% Simulation: This is the simulation first draft.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Version 2 adds thrusters for angular momentum %%%%%%%%%
%%%%%%%%% and a function that can solve for position if %%%%%%%%%
%%%%%%%%% a thruster is fired for some time.            %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

global Nframe
global dt
global T

%define simulation space frame
Nframe = [1 0 0; 0 1 0; 0 0 1];

T = 10;                      %duration of simulation
dt = 0.01;                   %time increment
t = 0;                       %initial time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% STUFF THAT WILL HAPPEN IN THE GUI %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define initial conditions
phi_0 = 0;  %initial spin orientation (Bframe x-axis)
psi_0 = 0;  %initial precession orientation
theta_0 = 0;  % The newtation angle, it is between the spin axis and the angular momentum vector
phi_dot = 0;  % initial rotation rate is zero (we will add rotation with a thruster)
% If and as forces act on the object to rotate it in the precession and
% newtation frames, the two below rates  will be defined and tracked as
% properties of the object
psi_dot = 0;
theta_dot = 0;
%define object and load its initial conditions into its properties
object1 = Cylinder;     %cylinder's mass properties are easy to define
object1.mass = 5; object1.radius = 4; object1.height = 20; object1.inertial_tensor = object1.CalculateInertialTensor;
object1.orientation = [phi_0; psi_0; theta_0];
%NOTE: Thruster definitions needs to be developed more. They are super
%buggy and don't work as intended.
object1.Thruster1 = [0.4 0 2 0];    %define moment arm and thruster force
object1.Thruster2 = [0.4 0 -2 0];    %define moment arm and thruster force
% define a cloud of points to track as I build this toolbox. Eventually
% this point cloud will not be needed, and we will only need mass
% properties and the locations of things like thrusters, pilots, fuel
% tanks, ammunition caches, etc.
N = 20;
[X,Y,Z] = cylinder([object1.radius object1.radius object1.radius],N);
%IMPORTANT: pointcloud() is a custom function which converts the awful
%output of cylinder() into a useable array of points. It is included in
%this commit, but typically I inject it into the toolbox folder.
object1.pointcloud = pointcloud(X,Y,Z*object1.height); object1.pointcloud(3,:) = object1.pointcloud(3,:) - object1.height/2;

% Rotate the currently upright cylinder to its defined initial orientation
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
Position = zeros(3,(N+1)*3,T*100+1);        %preallocate position vector as a function of the total simulation time. This vector is a history of the object.position's movement
n = 1;
Position(:,:,n) = object1.position;   %and define initial position
tic
while t <= T  %start at 2 for now, but eventually want it to be t_of_object_initialization + dt
      %%% ADD THE BELOW IN FOR A NICE MOVIE %%%
    plot3(Position(1,:,n),Position(2,:,n),Position(3,:,n),'b.'); axis equal; hold on;
    plot3(Position(1,N-1,n),Position(2,N-1,n),Position(3,N-1,n),'r-o'); hold off
    xlabel('x'); ylabel('y'); zlabel('z')
    text(0,object1.height,sprintf('Timestamp: %f',t))
    title('Motion of the object')
    M(n) = getframe(gcf);
              %%% END OF MOVIE CODE %%%
    t = t + dt;     %define the time stamp at which we will calculate
    n = n + 1;
    if t < 3                                 %for three seconds, no forces are applied
        [object1.orientation, object1.position] = object1.freerotate(dt,Nframe);    
    elseif (3 <= t) && (t <= 3.5)            %for a half a second, thruster 1 is fired
        [object1.spin_rates, object1.orientation, object1.position] = object1.firethruster(object1.Thruster1,dt,Nframe);    %here we fire a thruster, need to structure our method around this notation
    elseif (7.5 <= t) && (t <= 8)            %for a half a second, thruster 2 is fired (opposite direction)
        [object1.spin_rates, object1.orientation, object1.position] = object1.firethruster(object1.Thruster2,dt,Nframe);
    else                                     %for two seconds, no forces are applied
        [object1.orientation, object1.position] = object1.freerotate(dt,Nframe);
    end
    Position(:,:,n) = object1.position;
end
toc
disp('Final position of tracked point');
disp(object1.position(:,N-1));