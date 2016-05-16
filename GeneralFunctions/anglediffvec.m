function [ delta ] = anglediffvec(u)
% Finds the minimum difference between 2 angles qd (desired angle) qc (current angle) around the same axis 
% assuming they can rotate clockwise or anti-clockwise, current angle can
% be any size, desired is assumed to be between -180-180  in degrees or
% radians


% This function is slighly changed in order to deal with signals in
% simulink and to handle vector quantities



    qc=u(1:end/2);
    qd=u(end/2+1:end);

% Usage exlpained for degrees:
%Choose degrees by including any third argument

    
    qd=qd*180/pi;
    qc=qc*180/pi;

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


    delta=delta*pi/180;



end

