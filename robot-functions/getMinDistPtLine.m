function [ Pt,d,t ] = getMinDistPtLine( P1,P2,P3 )
%minDistancePointLine returns point of minimum distance between line and
%point
%   Line defined by two points P1 P2
%   Point at P3 




V12=P2-P1;
t=(V12'*(P3-P1))/(V12'*V12);
Pt=P1+t*V12;

d=norm(Pt-P3);

end

