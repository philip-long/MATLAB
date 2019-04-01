function y=rot2AngleAxis(T)
%For a given DCM output a representation


if size(T,1)==4 && size(T,2)==4
    R = T(1:3, 1:3);   % Extract rotation part of T
elseif size(T,1)==1 && size(T,2)==9
    R=reshape(T,3,3);
elseif size(T,1)==9 && size(T,2)==1
    R=reshape(T,3,3);
else
    R=T;
end

sx=R(1,1);
sy=R(2,1);
sz=R(3,1);
nx=R(1,2);
ny=R(2,2);
nz=R(3,2);
ax=R(1,3);
ay=R(2,3);
az=R(3,3);




Calpha=0.5*((sx+ny+az-1));
Salpha=0.5*(((nz-ay)^2+(ax-sz)^2+(sy-nx)^2)^0.5);
alpha=atan2(Salpha,Calpha);

if norm(alpha)<1e-13 %No change
            u=zeros(3,1);
elseif(Salpha<1e-13)     
    u=[signCon(nz-ay)*sqrt( (sx-Calpha)/(1-Calpha))
    signCon(ax-sz)*sqrt( (ny-Calpha)/(1-Calpha))
    signCon(sy-nx)*sqrt( (az-Calpha)/(1-Calpha))];
            
else            
        u= [(R(3,2)-R(2,3))/(2*Salpha)
            (R(1,3)-R(3,1))/(2*Salpha)
                (R(2,1)-R(1,2))/(2*Salpha)];
end
 
y=[alpha;u];
