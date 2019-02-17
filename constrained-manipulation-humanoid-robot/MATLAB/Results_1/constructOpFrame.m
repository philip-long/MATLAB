function [ T ] = constructOpFrame(p)
%constructOpFrame Construct a frame from planar coordinates

 
x=p(1);
y=p(2);
theta=p(3);
T=eye(4);
T(1,4)=x;
T(2,4)=y;
 T(3,4)=-0.090;
T(1:3,1:3)=[ cos(theta), -sin(theta), 0
             sin(theta),  cos(theta), 0
               0,           0, 1];
end

