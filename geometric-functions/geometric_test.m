
clear all,close all
clc

v1=[5*rand(2,1)-10;0]
v2=[5*rand(2,1)-10;0]
pt=[5*rand(2,1)-10;0]
d4 = getDistPointlineSeg(pt, v1, v2);
d3 = getDistPointline(pt, v1, v2)
pt3=getPointMinDistLinePoint(pt, v1, v2);
[pt4,d]=getPointMinDistSegPoint(pt, v1, v2);
[pt5,d,t]=minDistancePointLine(v1, v2,pt);
[ Pt,d ] = getMinDistPtSeg(v1, v2,pt )
norm(pt3-pt)
plot3([v1(1); v2(1)],[v1(2);v2(2)],[v1(3);v2(3)],'b-o')
hold on
plot3([pt(1)],[pt(2)],[pt(3)],'rx')
plot3([pt(1);pt3(1)],[pt(2);pt3(2)],[pt(3);pt3(3)],'r')
plot3([pt(1);pt4(1)],[pt(2);pt4(2)],[pt(3);pt4(3)],'k:')
axis('equal')

%%
% Find intersection point between plane formed by three points and a line

clear all,clc
% find plane normal
P1=[0;0;0];
P2=[1;0;0];
P3=[0;1;0];
Plane=[P1';P2';P3';P1'];

v32=P3-P2;
v21=P2-P1;
n1=skew(v32)*v21;

v31=P3-P1;
v21=P2-P1;
n2=skew(v31)*v21;

% 

Pa=[0.25;0.5;-2];
Pb=[0.5;0.5;-1];
Line=[Pa';Pb'];

[I,check]=plane_line_intersect(n1,P2,Pa,Pb)


plot3(Plane(:,1),Plane(:,2),Plane(:,3),'b') %Plot the surface
hold on
plot3(Line(:,1),Line(:,2),Line(:,3),'r-o') %Plot the surface
plot3(I(1),I(2),I(3),'rs','MarkerSize',10.,'MarkerEdgeColor','k','MarkerFaceColor',[0.5,0.5,0.5]) %Plot the surface
%%
P1=[0;0;0]
v1=[1;1;0]
P2=[0.5;5;0]
v2=[1;-0.2;0]
b=getIntersection2Lines(v1,P1,v2,P2)