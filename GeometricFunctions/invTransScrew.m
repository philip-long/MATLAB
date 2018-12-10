function ST_i=invTransScrew(S)
%screwTransform Summary of this function goes here
%   This transforms the a twist represent at point i to a twist at point j
%   Unlike screwTransformation, the frame of representation is assumed to be t
%   the same. 
ST_i=S;
ST_i(1:3,4:6)=S(4:6,1:3);
ST_i(4:6,1:3)=S(1:3,4:6);

end