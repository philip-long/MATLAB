function [ T ] = screwTransform(Pij)
%screwTransform Summary of this function goes here
%   This transforms the a twist represent at point i to a twist at point j
%   Unlike screTransformation, the frame of representation is assumed to be 
%   the same.  For instance we assume the input twist is already
%   represented in the ith frame. 
T=[eye(3) -skew(Pij)
    zeros(3) eye(3)];

end

