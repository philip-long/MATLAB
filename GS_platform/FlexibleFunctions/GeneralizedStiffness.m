function [ K_e ] = GeneralizedStiffness( q_e )
%STIFFNESS Summary of this function goes here
%   Detailed explanation goes here
global K 
K_e=[zeros(6,1);K.*q_e]

end

