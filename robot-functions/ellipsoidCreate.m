function [ hEllipse,vol  ] = ellipsoidCreate( J,shading,face_color)

if(nargin<2)
    shading=0.2;
    face_color=[0.3,0.3,0.3];
end

[s,v]=eig(J([1 2],:)*J([1 2],:)');


vol=abs(det(J(1:2,:)));
if(v(2,2)>v(1,1))
    ax2=s(:,1); l2=v(1,1);
    ax1=s(:,2); l1=v(2,2);
else
    ax1=s(:,1); l1=v(1,1);
    ax2=s(:,2); l2=v(2,2);
end

a=l1^0.5;
b=l2^0.5;
x0=0.0;
y0=0.0;
phi=atan(ax1(2)/ax1(1));
f='k';

%ELLIPSEDRAW can draw an arbitrary ellipse with given parameters.
%   The properties of that ellipse plot can be customized 
%   by setting the ellipse handle. 
%
%       hEllipse = ellipsedraw(a,b,x0,y0,phi,lineStyle)
%
%   Input parameters:
%       a           Value of the major axis
%       b           Value of the minor axis
%       x0          Abscissa of the center point of the ellipse
%       y0          Ordinate of the center point of the ellipse
%       phi         Angle between x-axis and the major axis
%       lineStyle   Definition of the plotted line style
%
%   Output:
%       hEllipse    Handle of the ellipse
%
%   Simple usage:
%       ellipsedraw(5,3);
%       ellipsedraw(5,3,'g--');
%       ellipsedraw(5,3,pi/4);
%
%   Complete usage:
%       h = ellipsedraw(5,3,1,-2,pi/4,'r-.');
%       set(h,'LineWidth',2);

% Designed by: Lei Wang, <WangLeiBox@hotmail.com>, 25-Mar-2003.
% Last Revision: 01-Apr-2003.
% Dept. Mechanical & Aerospace Engineering, NC State University.
% Copyright (c)2003, Lei Wang <WangLeiBox@hotmail.com>
%$Revision: 1.0 $  $ 4/1/2003 5:42:24 PM $

    lineStyle = 'k-';



theta = [-0.03:0.01:2*pi];

% Parametric equation of the ellipse
%----------------------------------------
 x = a*cos(theta);
 y = b*sin(theta);



% Coordinate transform 
%----------------------------------------
 X = cos(phi)*x - sin(phi)*y;
 Y = sin(phi)*x + cos(phi)*y;
 X = X + x0;
 Y = Y + y0;


% Plot the ellipse
%----------------------------------------
 hEllipse = plot(X,Y,lineStyle,'LineWidth',1.0);
 axis equal;
 fill(X,Y,face_color,'facealpha',shading)
 

end

