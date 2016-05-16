
% This script demonstartes how the orientation representations are
% intergrable when omega is converted to rep dot, this is an important
% script to test rotations and understand possible sources or errors




clear all,clc
t=0; % Time
tfinal=1.0;
Step=0.1
c=0; % Counter
omega=[0.2 ;0.7;0.3]; % Example Omega
Rinit=TransMat(pi/5,'rx','r')*TransMat(pi/6,'ry','r')*TransMat(pi/8,'rz','r')


%% 1. Rotation Matrix


Rt1=Rinit;
while t<tfinal
    t=t+Step;
    c=c+1;
    % Define OMEGA_cd
    
    Rdot=skew(omega)*Rt1;
    
    Rt2=(Rdot*Step)+Rt1;
    
    for i=1:3
        Rt2(i,:)=Rt2(i,:)/norm(Rt2(i,:));
    end
    Rt1=Rt2;
    
    
end

Rfinal_ByRotations=Rt1

%% 2. Euler Angles
Y=Rot_to_ZXZ(Rinit);
t=0;
phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

while t<tfinal
    t=t+Step;
    c=c+1;
    
    C=[-sin(phi1)*cot(theta1) ,  cos(phi1)*cot(theta1) ,     1
        cos(phi1)             ,  sin(phi1), 0
        sin(phi1)/sin(theta1) ,  -cos(phi1)/sin(theta1)  ,    0];
    
    Eulerdot=(C*omega*Step);
    
    phi1=phi1+Eulerdot(1);
    theta1=theta1+Eulerdot(2);
    psi1=psi1+Eulerdot(3);
end

Rfinal_ByEuler=ZXZ_to_Rot([phi1 theta1 psi1])


%% Quaternions
[Q1 Q2 Q3 Q4]=matrix2quaternion(Rinit); 
t=0;
Q=[Q1; Q2; Q3; Q4];

while t<tfinal
    t=t+Step;
    c=c+1;
    C=0.5*[  -Q(2) -Q(3) -Q(4)
        Q(1) Q(4) -Q(3)
        -Q(4) Q(1) Q(2)
        Q(3) -Q(2) Q(1)];
    
    Quaterniondot=C*omega*Step;
    Q=Q+Quaterniondot;
end

Rfinal_ByQuaternions=quaternion2matrix(Q)


%% Roll Pitch Yaw


Y=Rot_to_ZYX(Rinit);
t=0;

phi1=Y(1);
theta1=Y(2);
psi1=Y(3);

while t<tfinal
    t=t+Step;
    c=c+1;
    
     C=[cos(phi1)*tan(theta1) ,   sin(phi1)*tan(theta1), 1
            -sin(phi1)           ,   cos(phi1)      ,       0
            cos(phi1)/cos(theta1),sin(phi1)/cos(theta1),0];
    
    RPY=C*omega*Step;
    
    phi1=phi1+RPY(1);
    theta1=theta1+RPY(2);
    psi1=psi1+RPY(1);
end

Rfinal_ByRPY=ZYX_to_Rot([phi1 theta1 psi1])


%% Angle Axis

Y=Rot_to_AngleAxis(Rinit);
theta=Y(1);
u=Y(2:end);
t=0;
while t<tfinal
    t=t+Step;
    c=c+1;
    thetadot=(transpose(u))*omega;
    udot=(0.5*(((eye(3)-u*transpose(u))*cot(theta/2))-skew(u)))*omega;
    theta=theta+(thetadot*Step);
    u=u+(udot*Step);
    

end
K=[theta;u];
Rfinal_by_Angleaxis=AngleAxis_to_Rot(K)

