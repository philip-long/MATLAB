function E=getCartesianError(X)
% Finds the error in position and Orientation between 2 trasnformation
% matrix of vector inputs and outputs the following position and udelta
%   
% input: X, X(1:12)= [Position ; Orientation] of T
%           X(13:24)= [Position ; Orientation] of Td
%
% Other possible inputs X=[T Td]
%                       X=[T;Td]
%
%
%
% Ouput Err=[Position2-Position1
%            udelta(R2*R1')]
%
% Error from rotation matrix is referred to the world frame
% calculated by Rd*transpose(R)

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


%% Find Transformation matrix from data



% Option 1 input= [T1 T2]
if size(X,1)==4 && size(X,2)==8
    P=X(1:3,4);
    R=X(1:3,1:3);
    Pd=X(1:3,end);
    Rd=X(1:3,5:7);
elseif size(X,2)==4 && size(X,1)==8
% Option 1 input= [T1;T2]
    P=X(1:3,4);
    R=X(1:3,1:3);
    Pd=X(5:7,end);
    Rd=X(5:7,1:3);
elseif length(X)==24
% X=[P1;R1;P2;R2]
    P=X(1:3);
    R=reshape(X(4:12),3,3);
    Pd=X(13:15);
    Rd=reshape(X(16:24),3,3);
else
    disp 'I do nae recognize the input data, sorry!'
end

Pos_err=Pd-P;
DiffMat=Rd*(R'); % Rotation difference between desired and current frames
uAlpha=0.5*[DiffMat(3,2)-DiffMat(2,3);DiffMat(1,3)-DiffMat(3,1);DiffMat(2,1)-DiffMat(1,2)];



E=[Pos_err;uAlpha];


end
