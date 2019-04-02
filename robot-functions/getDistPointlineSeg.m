function d = getDistPointlineSeg(pt, v1, v2)
% This gives the minimum distance from a pt P to a line segmentwhich contains the
% points v1 and v2

if(max(size(pt)<3))
    pt(3)=0;
    v1(3)=0;
    v2(3)=0;
end
a = v1 - v2;
b = pt - v1;
detP=-b'*a;

if(detP<0)
    d=norm(v1-pt); % distance from endpoint 1
elseif(detP>norm(v2-v1)^2)
    d=norm(v2-pt); % distance from endpoint 2
else
    d = norm(cross(a,b)) / norm(a); % distance to line containing points v1 v2     
end

