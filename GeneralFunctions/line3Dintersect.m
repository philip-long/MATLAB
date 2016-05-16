function b = line3Dintersect(L1,P1,L2,P2 )
%line3Dintersect checks Intersection of two lines
%that are defined by a direction vector and a
%point 
%  b=0 skew
%  b=1 Intersect
%  b=2 parallel



    
    if size(L1,1)==1 % row vector
        if length(L1)==2  % 2D case append z coorindates
            L1=[L1,0]';P1=[P1,0]';
            L2=[L2,0]';P2=[P2,0]'; 
        else
            L1=L1';P1=P1';
            L2=L2';P2=P2';
        end
    else    
        if length(L1)==2  % 2D case append z coorindates
            L1=[L1;0];P1=[P1;0];
            L2=[L2;0];P2=[P2;0]; 
        end
    end

% Now we have two three 3D vectors
% Check parallel
if norm(cross(L1,L2))==0
    b=2;
end

% Check if they interstec

L1,L2,P1,P2



%


