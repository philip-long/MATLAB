function y=Rot2Rep(R)
% For a given DCM output a representation

global Rep

if size(R,1)==4
    R=R(1:3,1:3);%Transformation matrix
    
elseif size(R,1)==9 %R is a vector input
    R=reshape(R,3,3);
    
elseif size(R,1)==3 && size(R,2)==3
    %no command
else
    disp 'Not recognized'
end


Rep=3;
sx=R(1,1);
sy=R(2,1);
sz=R(3,1);
nx=R(1,2);
ny=R(2,2);
nz=R(3,2);
ax=R(1,3);
ay=R(2,3);
az=R(3,3);
switch Rep
    case 1 %Quaternions
        [Q1 Q2 Q3 Q4]=matrix2quaternion(R);
        y=[Q1 Q2 Q3 Q4];
    case 2 %Euler Angles Z X Z
        
        
        phi1=atan2(-ax,ay);
        phi2=phi1+pi;
        theta1=atan2((ax*sin(phi1))-(cos(phi1)*ay),az);
        theta2=atan2((ax*sin(phi2))-(cos(phi2)*ay),az);
        psi1=atan2((-cos(phi1)*nx)-sin(phi1)*ny,cos(phi1)*sx+sin(phi1)*sy);
        psi2=atan2((-cos(phi2)*nx)-sin(phi2)*ny,cos(phi2)*sx+sin(phi2)*sy);
        y=[phi1;theta1;psi1]'
        y=[phi2;theta2;psi2]'
    case 3% Angle Axis
        
        
        
        Calpha=0.5*(sx+ny+az-1);
        Salpha=0.5*(((nz-ay)^2+(ax-sz)^2+(sy-nx)^2)^0.5);
        alpha=atan2(Salpha,Calpha);
        
        if alpha==0 %No change
            
            u=zeros(3,1);
            
        else
            
            u= [(R(3,2)-R(2,3))/(2*Salpha)
                (R(1,3)-R(3,1))/(2*Salpha)
                (R(2,1)-R(1,2))/(2*Salpha)];
            
        end
        
        y=[alpha;u];
    case 4 % Roll Pitch Yaw
        
        
        phi1=atan2(sy,sx);
        phi2=phi1+pi;
        theta1=atan2(-sz,((cos(phi1)*sx)+(sin(phi1)*sy)));
        theta2=atan2(-sz,((cos(phi2)*sx)+(sin(phi2)*sy)));
        psi1=atan2((sin(phi1)*ax)-(cos(phi1)*ay),(-sin(phi1)*nx)+(cos(phi1)*ny));
        psi2=atan2((sin(phi2)*ax)-(cos(phi2)*ay),(-sin(phi2)*nx)+(cos(phi2)*ny));
        y=[phi1;theta1;psi1];
    case 5 % DCM matrix
        y=R;
        
end