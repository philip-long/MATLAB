% MATRIX2QUATERNION - Homogeneous matrix to quaternion
%
% Converts 4x4 homogeneous rotation matrix to quaternion
%
% Usage: Q = matrix2quaternion(T)
%
% Argument:   T - 4x4 Homogeneous transformation matrix
% Returns:    Q - a quaternion in the form [w, xi, yj, zk]
%
% See Also QUATERNION2MATRIX

% Copyright (c) 2008 Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% pk at csse uwa edu au
% http://www.csse.uwa.edu.au/
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

function Q = Rot_to_Quaternion(T)

% This code follows the implementation suggested by Hartley and Zisserman
if size(T,1)==4 && size(T,2)==4
    R = T(1:3, 1:3);   % Extract rotation part of T
elseif size(T,1)==1 && size(T,2)==9
    R=reshape(T,3,3);
elseif size(T,1)==9 && size(T,2)==1
    R=reshape(T,3,3);
else
    R=T;
end

% Find rotation axis as the eigenvector having unit eigenvalue
% Solve (R-I)v = 0;
[v,d] = eig(R-eye(3));

% The following code assumes the eigenvalues returned are not necessarily
% sorted by size. This may be overcautious on my part.
d = diag(abs(d));   % Extract eigenvalues
[s, ind] = sort(d); % Find index of smallest one
if d(ind(1)) > 0.001   % Hopefully it is close to 0
    warning('Rotation matrix is dubious');
end

axis = v(:,ind(1)); % Extract appropriate eigenvector

if abs(norm(axis) - 1) > .0001     % Debug
    warning('non unit rotation axis');
end

% Now determine the rotation angle
twocostheta = trace(R)-1;
twosinthetav = [R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)]';
twosintheta = axis'*twosinthetav;

theta = atan2(twosintheta, twocostheta);

Q = [cos(theta/2); axis*sin(theta/2)];
Q1=Q(1);
Q2=Q(2);
Q3=Q(3);
Q4=Q(4);
