function [ new_points ] = RN_a( points, psi )
%N_a Is a function that rotates a set of points around the first euler
%angle, psi.
%   Given a 3xn set of points and an angle, this function with use a matrix
%   to rotate that set of points around the origin. Make sure the angle is
%   in degrees.

N_a = [cosd(psi) sind(psi) 0;...
        -sind(psi) cosd(psi) 0;...
        0 0 1];
new_points = N_a*points;
end