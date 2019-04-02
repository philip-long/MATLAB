function [ delta ] = anglediff( qd,qc,deg)
% Finds the minimum difference between 2 angles qd (desired angle) qc (current angle) around the same axis 
% assuming they can rotate clockwise or anti-clockwise, current angle can
% be any size, desired is assumed to be between -180-180  in degrees or
% radians



% Usage exlpained for degrees:
%Choose degrees by including any third argument

if exist('deg','var')
    
else
    
    qd=qd*180/pi;
    qc=qc*180/pi;
end

% First eliminate over sizeed angles by mod  
qnc=mod(qc,360);
d1=qd-qnc;
d2=d1-(sign(d1)*360);
% There are two differences depening on recovering from anticlockwise
% or clockwise

if abs(d1)<=abs(d2)
    delta=d1;
else
    delta=d2;
end

if exist('deg','var')
    %disp 'degrees'
else
    %disp 'radians'
    delta=delta*pi/180;
end



end

