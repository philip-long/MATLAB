clear all,clc
earth_example;
radius=1;

[az,mz,r]=cart2sph(0.9511,0,0.309)
[x,y,z]=sph2cart(0.,0.3141,radius)

% Search for an element in X Y Z such that distance to this 
M=cezeros(size(x));
for i-1:size(x,1)
    for j=1:size(x,2)
        M(i,j)=
%
globe.CData(5,6,:)


[rx cx]=find(abs(globe.XData-x)<0.1)
[ry cy]=find(abs(globe.YData-y)<0.1)
[rz cz]=find(abs(globe.ZData-z)<0.1)
globe.XData(91,91)
globe.YData(91,91)
globe.ZData(91,91)
globe.CData(91,91,:)