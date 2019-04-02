function [point,intersect,ratio,vec_obj] = segRayIntersect(P1,P2,P3,v3,plotting)
%
%  segRayIntersect finds the intersect between a sgement and a ray segmnts if it
%  exists
% Input P1,P2 defines the segment
%       P3 and direction v3 is ray such that any point can be defined as
%   P3+t*v3 where t>0
%  -point is the intersection point of the two lines
%  -intersect defines if the line segment actually intersects
%  - ratio simply divides the line sgement in two according to where the
%  intersect point lies
%  e.g ratio_l1(1)=ratio_l1(2)=0.5 if intersect point lies halfway between
%  P1 P2
%  e.g ratio_l1(1)=1 ratio_l1(2)=0.0 if intersect point lies P1
% P1=cartesianVertex(k(i),:)
% P2=cartesianVertex(k(i+1),:)
% P3=P_origin
% v3=ObjectVectorinPlane
%
%
%
%
if(size(P1,1)<size(P1,2)) P1=P1'; end
if(size(P2,1)<size(P2,2)) P2=P2'; end
if(size(P3,1)<size(P3,2)) P3=P3'; end
if(size(v3,1)<size(v3,2)) v3=v3'; end
ratio=zeros(2,1);

[point,b]=linlinintersect(P1, (P2-P1)/norm(P2-P1),P3,v3/norm(v3));




if(b==1)
    % intersecting
    t=(point-P1)./ ((P2-P1));
    t2=(point-P3)./ ((v3));
    t=t(~isnan(t));
    t=t(~isinf(t));
    t2=t2(~isnan(t2));
    if(isempty(t))
        intersect=0;
        ratio(1)=0;
        ratio(2)=0;    
    elseif(t(1)>1 || t(1)<-0.00000001)
        % Outside segment
        intersect=0;
        ratio(1)=0;
        ratio(2)=0;
    else
        intersect=1;
        ratio(1)=t(1);
        ratio(2)=1-t(1);
    end
    if(isempty(t2))
        intersect=0;
        ratio(1)=0;
        ratio(2)=0; 
    elseif(t2(1)<0)
        % not in ray directopm
        intersect=0;
        ratio(1)=0;
        ratio(2)=0;
    end
    
else
     intersect=0;
    ratio(1)=0;
    ratio(2)=0;

end






if(nargin==5 && plotting==true && size(P1,1)==2)
    plot([P1(1),P2(1)],[P1(2),P2(2)],'b-o')
    hold on
    plot([P3(1),P3(1)+v3(1)],[P3(2),P3(2)+v3(2)],'y-o')
    plot(point(1),point(2),'s','MarkerEdgeColor','k','MarkerFaceColor',[0.5,0.5,0.5]);
end

if(nargin==5 && plotting==true && size(P1,1)==3 && intersect)
    plot3([P1(1),P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'g-o','LineWidth',3.0)
    hold on
    plot3([P3(1),P3(1)+v3(1)],[P3(2),P3(2)+v3(2)],[P3(3),P3(3)+v3(3)],'y-o')
    plot3(point(1),point(2),point(3),'s','MarkerEdgeColor','k','MarkerFaceColor',[0.5,0.5,0.5]);
end

end