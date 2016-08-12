function T=TransMat(y,Type,Out)

%% Transmat finds the transformation matrix for a simple rotation or translation.
%
%The choices are deined as follows:
%    T=TransMat(y,Type)
%  y can be symbolic or numeric polymorphous to a certain extent
%
% ROTATION:
%    y=theta
%    'Type'=Rx or RX or x or rx
%    'Type'=Ry or RY or y or ry
%    'Type'=Rz or RZ or z or rz
% TRANSLATION
%   y=[x,y,z]
%   'Type'=
%
% OUTPUT
% dim of T can be selected by Out
%  Out==T or ~
%       T=4x4
%  Out==R  or rot or r    rotation matrix only
%       T=3x3
%  Out==P   or t or trans   translation part only
%       T=3x1


if length(y)==1 % Rotation input
    
    switch Type
        
        case {'Rx','RX','x','rx'}
            %disp 'rotation around',Type,'axis'
            T=[1        0       0       0
                0        cos(y) -sin(y)  0
                0        sin(y)  cos(y)  0
                0        0       0       1];
        case {'Ry','RY','y','ry'}
            %disp 'rotation around',Type,'axis'
            T=[cos(y)  0      sin(y) 0
                0       1      0      0
                -sin(y) 0      cos(y) 0
                0       0      0      1];
        case {'Rz','RZ','z','rz'}
            %disp 'rotation around',Type,'axis'
            T=[cos(y) -sin(y)    0    0
                sin(y)  cos(y)    0    0
                0       0         1    0
                0       0         0    1];
        otherwise
            disp 'Invalid rotation input must be Rx, Ry Rz'
    end
    
elseif length(y)==3 %Translation Input
    
    % disp 'Translation'
    T=[1 0 0 y(1)
        0 1 0 y(2)
        0 0 1 y(3)
        0 0 0 1];
else
    
    disp ' Improper input,'
    disp 'y has one entry for rotation and 3 entries for translation'
    
    
end

% Define the output of the Matrix

if exist('Out','var')
    switch Out
        case {'R','rot','r'}
            T=T(1:3,1:3);
        case {'P','t','trans'}
            T=T(1:3,4);
        otherwise
            %T does not change
    end
end



% Copyright (c) 2012 Philip Long
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.