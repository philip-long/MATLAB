function Vj=ScrewTransformation(T,V)
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
% This function Transfroms a kinematic screw V
% which is measured at origin i in the frame i to
% the same kinematic screw measured at origin j in
% the frame j
% 
%  Vjj=ScrewTransformation*Vii
%
%
% Inputs: ScrewTransformation(Tij,Vii)
% 1. Transformation matrix from point j to point i
%  4 x 4 trasnformation Matrix Tij
% 2. Kinematic screw expressed at point origin i
% in the frame of i, Vii
%   6x1 Velocity screw
% 
% Ouput:  Kinematic screw expressed at point origin j
% in the frame of j,  Vjj

% Unpack the variables
Tij=T;
Vi=V;


Pij=Tij(1:3,4);
Rij=Tij(1:3,1:3);
Rji=transpose(Rij);


T=[Rji -Rji*skew(Pij)
    zeros(3) Rji];

Vj=T*Vi;