function [ Vec1,Vec2 ] = dis2visionline(u)
%dis2visionline This function calculates the
%vecotors of minimum distacne from one point to
%two line segments
%  
% Input: VP1,VP2,Pv,Pc
% Output: Vec1--> point on line |Pv VP1| closest on to Pc 
%         Vec2--> point on line |Pv VP2| closest on to Pc 
%
% Line 1 = |Pv VP1|
% Line 2 = |Pv VP2|
% 

% Line 1 , using inf as cylinder radius so really
% the function just checks if point is between two
% planes

VP1=u(1:3);VP2=u(4:6);Pv=u(7:9);Pc=u(10:12);


[InCylinder_Flag,R] = pt_in_cylinder( VP1,Pv,Inf,Pc );

if InCylinder_Flag==0 % not between two planes
    
    if norm(Pc-VP1)<norm(Pc-Pv)
        Vec1=VP1;
        %VP1 is closest point
    else
        Vec1=VP2;
        %VP2 is closest point
    end
    
else


    t=-((Pv-Pc)'*(VP1-Pv))/sum((VP1-Pv).^2);
    Vec1=Pv+(VP1-Pv)*t;
    % Checks (a) Orthogonality, Distance=R 
    % (Pv-VP1)'*(Vec1-Pc) must equal zero
    % norm(Vec1-Pc) =R 
    
end

% Line 2 , using inf as cylinder radius so really
% the function just checks if point is between two
% planes
[InCylinder_Flag,R] = pt_in_cylinder( VP2,Pv,inf,Pc );

if InCylinder_Flag==0 % not between two planes
    
    if norm(Pc-VP2)<norm(Pc-Pv)
        Vec2=VP1;
        %VP1 is closest point
    else
        Vec2=VP2;
        %VP2 is closest point
    end
    
else
    
    t=-((Pv-Pc)'*(VP2-Pv))/sum((VP2-Pv).^2);
    Vec2=Pv+(VP2-Pv)*t;
    % Checks (a) Orthogonality, Distance=R 
    % (Pv-VP2)'*(Vec2-Pc) must equal zero
    % norm(Vec2-Pc) 
    
end



end

