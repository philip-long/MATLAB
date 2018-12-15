function [point,b,t] = getIntersectionLines(P1,v1,P2,v2)

%line3Dintersect checks Intersection of two lines
%that are defined by a direction vector and a
%point
%  b=0 skew
%  b=1 Intersect
%  b=2 parallel
%
%
% P1=P1
% v1=(P2-P1)/norm(P2-P1),
% P2=P3
% v2=v3/norm(v3)

% 2D case
A=[P1,P1+v1,P2,P2+v2];
if( all(A(3,:)==0) )
    lines=A(1:2,:)';
     % lines=[P1(1:2)';P1(1:2)+v1(1:2);P2(1:2);P2(1:2)+v2(1:2)];
    point=getIntersection2DLines(lines);    
    if(any(isnan(point)))
        b=2;
    else
        b=1;
    point=[point' ;0]; % Philip Changing this to a col vector
    end   
else
    point=zeros(3,1);
    
    % 3D
    % Check if parallel
    if norm(cross(v1/norm(v1),v2/norm(v2)))==0
        b=2;
    else        
        sol=P2-P1;
        st1=([v1(1:2), -v2(1:2)])\sol(1:2);
        st3=([v1(2:3), -v2(2:3)])\sol(2:3);        
        if(norm(st1-st3)<0.0000001)
            b=1;
            point=P1+st1(1)*v1;
        else
            b=0;
        end
    end
    
end
end