function [ d ] = getMinObjectDistance( r2,q3,link_name,object )
%getMinObjectDistance Minimum distance from link to object

p=getTransform(r2,q3,link_name);
obj=object.project(p(1:3,4));
d=obj.dist;


end

