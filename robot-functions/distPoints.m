function d= distPoints( Px,Py,Pz)
%distPoints Distance between  points
%   
%   one arguement : distance to origin
%   two arguments : planar distance between points
%   three arguments : spatial distance between points
switch nargin
    case 1 % one entry
        d=norm(Px);
    case 2
        d=((Px(1)-Px(2))^2+(Py(1)-Py(2))^2)^0.5;
    case 3        
        d=((Px(1)-Px(2))^2+(Py(1)-Py(2))^2+(Pz(1)-Pz(2))^2)^0.5;
    otherwise
         ME = MException('MyComponent:noSuchVariable', ...
        'not enough arguments');
    throw(ME)
end

