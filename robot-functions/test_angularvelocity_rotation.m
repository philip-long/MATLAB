
% This script demonstartes how the orientation representations are
% intergrable when omega is converted to rep dot, this is an important
% script to test rotations and understand possible sources or errors




clear all,clc
t=0; % Time
tfinal=1.0;
Step=0.1
c=0; % Counter
omega=[0.2 ;0.7;0.3]; % Example Omega
angles3=randRange(-pi/4,pi/4,3);
Rinit=expm(skew([1;0;0]*(1)))*expm(skew([0;1;0]*angles3(2)))*expm(skew([0;0;1]*angles3(3)));



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

Rfinal_ByRotations=Rt1;

%% 2. Euler Angles
Y=rot2Euler_zxz(Rinit);
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

Rfinal_ByEuler=zxz_euler2Rot([phi1 theta1 psi1]);


%% Quaternions
Q=rot2Quat_wxyz(Rinit); 
t=0;
Q=[Q(1); Q(2); Q(3); Q(4)];

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

Rfinal_ByQuaternions=wxyz_quat2rot(Q);


%% Roll Pitch Yaw


Y=rot2Euler_zyx(Rinit);
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

Rfinal_ByRPY=zyx_euler2Rot([phi1 theta1 psi1]);


%% Angle Axis

Y=rot2AngleAxis(Rinit);
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
Rfinal_ByAngleaxis=angleAxis_2Rot(u,theta);


disp ' Comparison of integration methods'
Rfinal_ByRotations
Rfinal_ByAngleaxis
Rfinal_ByQuaternions
Rfinal_ByRPY
Rfinal_ByEuler
pause()

%% An example how to obtain a correcting angular velocity term from rotation matrices

clear all,clc
% Three random angles and axes
angles=randRange(-pi,pi,1);
u1=randRange(-1,1,3); u1=u1/norm(u1);

deviation=randRange(-pi/12,pi/12,1);
deviation_axis=randRange(-1,1,3); deviation_axis=deviation_axis/norm(deviation_axis);

wR1=expm(skew(u1)*angles);
wR2=expm(skew(u1)*angles)*expm(skew(deviation_axis)*deviation);

% Khalil: rot(u,alpha)*wRe=wRedesired
%  rot(u,alpha)=wRedesired* transpose(wRe);

wRwe=wR1*wR2'; % Error between the in world frame
rot_u_alpha=rot2AngleAxis(wRwe);
angular_velocity=rot_u_alpha(1)*rot_u_alpha(2:4);

Euler_dot=angVelocity2EulerZXZDot(angular_velocity,wR1);
RPY_dot=angVelocity2EulerZYXDot(angular_velocity,wR1);
u_alpha_dot=angVelocity2AngleAxisdot(angular_velocity,wR1);
quat_dot=angVelocity2Quaterniondot(angular_velocity,wR1);


% Alternatively by RPY axis

rpy2=rot2Euler_zyx(wR2);
rpy1=rot2Euler_zyx(wR1);
rpy1-rpy2;
phi1=rpy1(1); theta1=rpy1(2); psi1=rpy1(3);

Cinv=[0 -sin(phi1) cos(phi1)*cos(theta1)
      0  cos(phi1) sin(phi1)*cos(theta1) 
      1   0             -sin(theta1)];

angular_velocity_by_rpy=Cinv*(rpy1-rpy2);


% Alternatively by Euler angles
e2=rot2Euler_zxz(wR2);
e1=rot2Euler_zxz(wR1);
e1-e2;
phi1=e1(1); theta1=e1(2); psi1=e1(3);

Cinv=[0 cos(phi1) sin(phi1)*sin(theta1)
      0  sin(phi1) -cos(phi1)*sin(theta1) 
      1   0             cos(theta1)];
  
angular_velocity_by_euler=Cinv*(e1-e2);

% Alternatively by quaternion
Q2=rot2Quat_wxyz(wR2);
Q1=rot2Quat_wxyz(wR1);
    C=0.5*[  -Q1(2) -Q1(3) -Q1(4)
        Q1(1) Q1(4) -Q1(3)
        -Q1(4) Q1(1) Q1(2)
        Q1(3) -Q1(2) Q1(1)];

    % Left inverse Khalil 5.69
angular_velocity_by_quaternions=4*transpose(C)*(Q1-Q2); 


fprintf('Error using quaternions = %d \n', norm(angular_velocity_by_quaternions-angular_velocity)); 
fprintf('Error using RPY (ZYX) angle = %d \n', norm(angular_velocity_by_rpy-angular_velocity));
fprintf('Error using Euler (ZXZ) angle = %d \n \n', norm(angular_velocity_by_euler-angular_velocity));

fprintf('Error quaternion dot reconstruction = %d \n', norm(quat_dot-(Q1-Q2)));
fprintf('Error RPY dot reconstruction = %d \n', norm(RPY_dot- (rpy1-rpy2)));
fprintf('Error Euler dot reconstruction = %d \n', norm(Euler_dot- (e1-e2)));


