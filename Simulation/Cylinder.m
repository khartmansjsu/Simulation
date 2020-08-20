classdef Cylinder
   properties
      mass                          %mass
      height                        %height
      radius                        %radius
      inertial_tensor               %[3x3] mass properties matrix
      position                      %[x,y,z] object's current point cloud
      orientation                   %[psi;phi;theta]
      pointcloud                    %[x,y,z] object's initial position
      spin_rates                    %[psi_dot;phi_dot;theta_dot]
      reaction_wheel_spin           %angular velocity vector IN B FRAME [x1; x2; x3] deg/s
      reaction_wheel_I              %[3x3] mass properties matrix
      Thruster1                     %a thruster whose Force vector is defined by the user [[Force] [Location of thruster]]
      Thruster2
   end
   methods
      function I = CalculateInertialTensor(obj)
          if isempty(obj.mass) == 1 || isempty(obj.height) == 1 || isempty(obj.radius) == 1
              disp('Cannot define inertial tensor because mass and/or dimensions are not defined.')
          elseif isempty(obj.inertial_tensor) == 1
              I = [(obj.mass/12)*(3*obj.radius^2+obj.height^2) 0 0;
              0 (obj.mass/12)*(3*obj.radius^2+obj.height^2) 0;
              0 0 (obj.mass*obj.radius^2)/2];
          else
              disp('Inertial tensor already defined.')
          end
      end
      function [new_orientation, position] = freerotate(obj,dt,Nframe)
          new_orientation = [obj.orientation(1) + obj.spin_rates(1)*dt;...
                             obj.orientation(2) + obj.spin_rates(2)*dt;...
                             obj.orientation(3) + obj.spin_rates(3)*dt];
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%% now we have angles for time stamp we are in %%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          delta = new_orientation - obj.orientation;    %going to keep this value handy for feedback loops
          % rotate
          R_an = [cosd(new_orientation(1)) sind(new_orientation(1)) 0;        %Precession rotation matrix
                  -sind(new_orientation(1)) cosd(new_orientation(1)) 0;
                  0 0 1];
          R_ga = [1 0 0;                                                       %Newtation rotation matrix
                  0 cosd(new_orientation(3)) sind(new_orientation(3));     
                  0 -sind(new_orientation(3)) cosd(new_orientation(3))];
          R_bg = [cosd(new_orientation(2)) sind(new_orientation(2)) 0;        %Spin rotation matrix
                  -sind(new_orientation(2)) cosd(new_orientation(2)) 0;
                  0 0 1];
          R_bn = R_bg*R_ga*R_an;                %  (rotates to b from n)
          Bframe = R_bn.'*Nframe;                 %Define B frame in terms of new angles and rotation matrices
          % w = psi_dot*N_x + phi_dot*B_x;
          % I = [(m/12)*(3*r^2+h^2) 0 0;                 %Check if mass properties matrix is constant
          % 0 (m/12)*(3*r^2+h^2) 0;
          % 0 0 (m*r^2)/2];                              %constant **
          position = Bframe*obj.pointcloud;  %update position property as a function of initial position and current orientation
      end
      function [w, new_orientation, position] = firethruster(obj,Thruster,dt,Nframe)
          %calculate the moment from the moment arm of the thruster being
          %fired, and the force that the thruster exerts
          M = cross(Thruster(:,2),Thruster(:,1));
          %intermediate calculation of the matrix which relates rotational
          %rate of Bframe to the rotational rate of the Euler angles (if
          %theta is 0, we have some problems.)
          Invert = 1/sind(obj.orientation(3))*[sind(obj.orientation(2)) cosd(obj.orientation(2)) 0;...
                                               -sind(obj.orientation(2))*cosd(obj.orientation(3)) -cosd(obj.orientation(2))*cosd(obj.orientation(3)) sind(obj.orientation(3));...
                                               cosd(obj.orientation(2))*sind(obj.orientation(3)) -sind(obj.orientation(2))*sind(obj.orientation(3)) 0];
          w = Invert*dt*(obj.inertial_tensor.'*M) + obj.spin_rates;
          new_orientation = [obj.orientation(1) + w(1)*dt;...
                             obj.orientation(2) + w(2)*dt;...
                             obj.orientation(3) + w(3)*dt];
          delta = new_orientation - obj.orientation;    %going to keep this value handy for feedback loops
          %now that we've tracked what has happened to the object, redefine
          %Bframe in terms of Nframe and new rotation matrix
          R_an = [cosd(new_orientation(1)) sind(new_orientation(1)) 0;        %Precession rotation matrix
              -sind(new_orientation(1)) cosd(new_orientation(1)) 0;
              0 0 1];
          R_ga = [1 0 0;                                                       %Newtation rotation matrix
                  0 cosd(new_orientation(3)) sind(new_orientation(3));
                  0 -sind(new_orientation(3)) cosd(new_orientation(3))];
          R_bg = [cosd(new_orientation(2)) sind(new_orientation(2)) 0;        %Spin rotation matrix
                  -sind(new_orientation(2)) cosd(new_orientation(2)) 0;
                  0 0 1];
          R_bn = R_bg*R_ga*R_an;                %  (rotates to b from n)
          Bframe = R_bn.'*Nframe;                 %Define B frame
          %update the position of our object (an array of points) and the
          %position of the thruster as functions of initial positions and
          %new object Bframe orientation
          position = Bframe*obj.pointcloud;
      end
   end
end