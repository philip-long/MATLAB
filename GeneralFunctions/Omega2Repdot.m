function Repdot=Omega2Repdot(u)

Omega=u(1:3);
R=u(4:12);

R=reshape(R,3,3);
%R=TransMat(-pi/4,'z','rot')


C=u(13);
% Angle axis
% ut1=vrrotmat2vec(R1)
% ut2=vrrotmat2vec(R2)
% ut=vrrotmat2vec(Rd)
% vrrotvec2mat(ut)

% Quaternion
%1 My Code

switch C
    case 1 %Quarternions
        %Q1=0.5*(trace(R)+1)^(0.5)
        %Q2=0.5*sign(R(3,2)-R(2,3))*(R(1,1)-R(2,2)-R(3,3)+1)^(0.5)
        %Q3=0.5*sign(R(1,3)-R(3,1))*(-R(1,1)+R(2,2)-R(3,3)+1)^(0.5)
        %Q4=0.5*sign(R(2,1)-R(1,2))*(-R(1,1)-R(2,2)+R(3,3)+1)^(0.5)
        
        % 2 Code from Internet that has all bells and whistles
        
        [Q1 Q2 Q3 Q4]=matrix2quaternion(R);
        Q=[Q1 Q2 Q3 Q4];
        % Qdot=C*Omega
        C=0.5*[  -Q2 -Q3 -Q4
            Q1 Q4 -Q3
            -Q4 Q1 Q2
            Q3 -Q2 Q1];
        
        
        Repdot=C*Omega; %Qdot = C omega
    case 2 % Euler Angles
        sx=R(1,1);
        sy=R(2,1);
        sz=R(3,1);
        nx=R(1,2);
        ny=R(2,2);
        nz=R(3,2);
        ax=R(1,3);
        ay=R(2,3);
        az=R(3,3);
        
        phi1=atan2(-ax,ay);
        phi2=phi1+pi;
        theta1=atan2((ax*sin(phi1))-(cos(phi1)*ay),az);
        theta2=atan2((ax*sin(phi2))-(cos(phi2)*ay),az);
        psi1=atan2((-cos(phi1)*nx)-sin(phi1)*ny,cos(phi1)*sx+sin(phi1)*sy);
        psi2=atan2((-cos(phi2)*nx)-sin(phi2)*ny,cos(phi2)*sx+sin(phi2)*sy);
        Euler_angs=[phi1;theta1;psi1];
        
        REuler=TransMat(phi1,'z','rot')*TransMat(theta1,'x','rot')*TransMat(psi1,'z','rot');
        REuler2=TransMat(phi2,'z','rot')*TransMat(theta2,'x','rot')*TransMat(psi2,'z','rot');
        
        
        C=[-sin(phi1)*cot(theta1) ,  cos(phi1)*cot(theta1) ,     1
            cos(phi1)             ,  sin(phi1), 0
            sin(phi1)/sin(theta1) ,  -cos(phi1)/sin(theta1)  ,    0];
        
        Repdot=C*Omega; %Qdot = C omega
    case 3
        Repdot=skew(Omega)*R;
    case 4 %Roll Pitch Yaw
        sx=R(1,1);
        sy=R(2,1);
        sz=R(3,1);
        nx=R(1,2);
        ny=R(2,2);
        nz=R(3,2);
        ax=R(1,3);
        ay=R(2,3);
        az=R(3,3);
        
        phi1=atan2(sy,sx);
        phi2=phi1+pi;
        theta1=atan2(-sz,((cos(phi1)*sx)+(sin(phi1)*sy)));
        theta2=atan2(-sz,((cos(phi2)*sx)+(sin(phi2)*sy)));
        psi1=atan2((sin(phi1)*ax)-(cos(phi1)*ay),(-sin(phi1)*nx)+(cos(phi1)*ny));
        psi2=atan2((sin(phi2)*ax)-(cos(phi2)*ay),(-sin(phi2)*nx)+(cos(phi2)*ny));
                
        RRPY=TransMat(phi1,'z','rot')*TransMat(theta1,'y','rot')*TransMat(psi1,'x','rot');
        RRPY=TransMat(phi2,'z','rot')*TransMat(theta2,'y','rot')*TransMat(psi2,'x','rot');
        
        C=[cos(phi1)*tan(theta1) ,   sin(phi1)*tan(theta1), 1
            -sin(phi1)           ,   cos(phi1)      ,       0
            cos(phi1)/cos(theta1),sin(phi1)/cos(theta1),0];
        
        Repdot=C*Omega; %Qdot = C omega
          
       
        
    case 5 % Angle Axis
        Y=Rot_to_AngleAxis(Rinit);
        theta=Y(1);
        u=Y(2:end);
        thetadot=transpose(u)*Omega;
        udot=0.5*(((eye(3)-u*tranpose(u))*cot(theta/2))-skew(u))
        
end

























