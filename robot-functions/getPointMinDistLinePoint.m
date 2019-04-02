function [ pt_min ] = getPointMinDistLinePoint(pt, v1, v2)
%getPointMinDistLinePoint returns the point on line defined by v1 v2 of
%minimum distance to point p1
%   Detailed explanation goes here

if(max(size(pt)<3))
    pt(3)=0;
    v1(3)=0;
    v2(3)=0;
end

b = pt - v1;
a = v2 - v1;
proj =(b'*a)/(a'*a);
pt_min=v1+proj*a;


end

