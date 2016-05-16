function d = point_to_line(pt, v1, v2)
% This gives the minimum distance from a pt P to a line which contains the
% points v1 and v2
a = v1 - v2;
b = pt - v2;
d = norm(cross(a,b)) / norm(a);