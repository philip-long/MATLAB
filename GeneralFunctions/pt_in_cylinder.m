function [ InCylinder_Flag,R ] = pt_in_cylinder( C1,C2,Radius,TCP )
%pt_in_cylinder Returns the wheather a point is in a cylinder or not,
% optional output arguement distance to cylinder




% C1 is first cap of cylinder
% C2 is second cap of cyinder
% Radius of the cylinder
% TCP is the point we're checking

N=(C2-C1)/norm(C2-C1);
% If P is between two planes, Test1 and Test2 must have opposite sides
% Therefore using the dot product a b cos(<ab), can find which side on
% plane point is on
Test1=((TCP-C1)'*N); % Check if P is on positive or negative side of plane
Test2=((TCP-C2)'*N); % Check if P is on positive or negative side of plane

% Finding the minimum distance (perpendiculr distance) to the line from the
% point 
R=point_to_line(TCP, C1, C2);


if (Test1*Test2)<=0 && R<Radius% if the point is between or on either plane
        InCylinder_Flag=1;
else
        InCylinder_Flag=0;
end


end

