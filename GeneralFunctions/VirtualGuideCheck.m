function InCylinder_Flag=VirtualGuideCheck(u)

% This function finds if a point lies in a cylinder, defined by the C1, C2
% which are point on the axis at each cap, and r defining the radius
% 
% Returns InCylinder_Flag=1 if in Cylinder
% Returns InCylinder_Flag=0 if outside cylinder planes
% Returns InCylinder_Flag=R-r between planes bu outside radius
global Guide_Radius
q=u(1:7);
C1=u(8:10);
C2=u(11:13);

% Find T70
T=T70(q);
TCP=T(1:3,4);



if length(C1)==4 % in homogenous coordiantes
    C1(end)=[];
    C2(end)=[];
end
% Defines axis, i.e normal to both caps
N=(C2-C1)/norm(C2-C1);
% If P is between two planes, Test1 and Test2 must have opposite sides
% Therefore using the dot product a b cos(<ab), can find which side on
% plane point is on
Test1=((TCP-C1)'*N); % Check if P is on positive or negative side of plane
Test2=((TCP-C2)'*N); % Check if P is on positive or negative side of plane

% Finding the minimum distance (perpendiculr distance) to the line from the
% point 
R=point_to_line(TCP, C1, C2);


if (Test1*Test2)<=0% if the point is between or on either plane
    if R<Guide_Radius
        InCylinder_Flag=1;
    else
        InCylinder_Flag=R-Guide_Radius;
    end
else
    InCylinder_Flag=0;
end

