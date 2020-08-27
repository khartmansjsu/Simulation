function pointcloud = pointcloud(X,Y,Z)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% TAKE X Y Z OUTPUTS FROM A MATLAB FUNCTION LIKE %%%
%%% CYLINDER OR SPHERE AND GENERATE A 3 X N MATRIX %%%
%%%    OF POINTS. EACH POINT IS POINTCLOUD(:,N)    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Example:

%    >> [X,Y,Z] = cylinder([0.5 0.25 0.5 0.25 0.5],40)
%    >> surf(X,Y,Z)         to see the mutant cylinder
%    >> plot3(X,Y,Z,'.')    to see the rainbow points
%    >> pointcloud(X,Y,Z)   to generate a 3x1681 double of points
%    >> ans(:,n)            to extract nth point in the cloud

%throw an error if number of arguments is less than 2 or greater than 3
%eventually want to make this less than 1, but I have no use of converting
%a 1-D meshgrid to a line, so eh.
narginchk(2,3)
if exist('Z', 'var')
    interim(:,:,1) = X; interim(:,:,2) = Y; interim(:,:,3) = Z;
    dims = size(interim);
    pointcloud = reshape(interim,[dims(1)*dims(2),3]);
    pointcloud = pointcloud';
else
    Z = 0;
    interim(:,:,1) = X; interim(:,:,2) = Y; interim(:,:,3) = Z;
    dims = size(interim);
    pointcloud = reshape(interim,[dims(1)*dims(2),3]);
    pointcloud = pointcloud';
end 
end