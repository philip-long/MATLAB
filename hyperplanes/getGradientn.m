function [dndq] = getGradientn(v1,v2,dv1,dv2)
%getGradientn get gradient of normalized cross product 
%  
    u=cross(v1,v2);
    v=norm(cross(v1,v2));
    %n=u/v;
    
    dudq=-skew(v2)*dv1 + skew(v1)*dv2;
    dvdq=(dudq'*u)/((u'*u)^0.5);
    
    
    dndq= (dudq*v - u*dvdq )/ v^2;
end

