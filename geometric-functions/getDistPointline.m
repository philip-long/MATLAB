function d = getDistPointline(pt, v1, v2)
% This gives the minimum distance from a pt P to a line which contains the
% points v1 and v2
if(max(size(pt)<3))
    pt(3)=0;
    v1(3)=0;
    v2(3)=0;
end
a = v1 - v2;
b = pt - v2;
d = norm(cross(a,b)) / norm(a);
