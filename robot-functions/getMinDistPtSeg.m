function [ Pt,d ] = minDistancePointSegment( P1,P2,P3 )
%minDistancePointLine returns point of minimum distance between line and
%point
%   Line defined by two points P1 P2
%   Point at P3
if(size(P1,1)<size(P1,2)) P1=P1'; end
if(size(P2,1)<size(P2,2)) P2=P2'; end
if(size(P3,1)<size(P3,2)) P3=P3'; end

[ Pt,d,t] = getMinDistPtLine( P1,P2,P3 );

if(t<0)
    disp 'Min distance is not within segment'
    Pt=P1;
    d=norm(P1-P3);
elseif(t>1)
    disp 'Min distance is not within segment'
    Pt=P2;
    d=norm(P2-P3);
end



end





