function JE=GENJAC(q,S)
%% GENJAC(q,S)

% q is the array of joints, S is the point we want. 

% If we want the velocity of the terminal joint just pick S=n (6 generally)
% If we want the the same velocity but at the end effector frame, S is ommited but the tool transfomration is
% required as global variable TnE. 

% Finallly if S is less than the last joint for example the 2 joint
% the jacobian gives the velocity of the robot up to the second joint at the second joint;
% All jacobians are refered to the world frame


global alpha d theta r sigma Joints TnE

%If we want the jacobian for a different frame S=frame
if exist('S','var')
     n=S;
else
     n=Joints;
end

T0n=eye(4);
J0n=zeros(6,n);

for j=1:n
    if sigma(j)==1
        rr(j)=q(j)+r(j);
        t(j)=theta(j);
    else
        t(j)=q(j)+theta(j);
        rr(j)=r(j);
    end
    Tj=[cos(t(j)) -sin(t(j)) 0 d(j)
        cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
        sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
        0 0 0 1];
    T0n=T0n*Tj;
    p=T0n(1:3,4); % For each Transformation Matrix extract Displacement Px Py Pz
    a=T0n(1:3,3); % For each Transformation Matrix extract ax ay az
    AJ(:,j)=a;
    PJ(:,j)=p;
end


for k=1:n
    askew=[0        -AJ(3,k) AJ(2,k)
        AJ(3,k)   0        -AJ(1,k)
        -AJ(2,k)  AJ(1,k)       0];
    J0n(:,k)=[sigma(k)*AJ(:,k)+(1-sigma(k))*askew*(PJ(:,n)-PJ(:,k))
        (1-sigma(k))*AJ(:,k)];
end

if ~exist('S','var')
 T=GENDGM(q);
 T0T3=T{end};
 try
    P=TnE(1:3,4);
 catch
    disp 'S is not picked=> you want J @ TCP but TnE is not defined!'
    disp 'Therefore finding giving Jacobian at last joint'
    P=[0;0;0];
 end
 L=T0T3(1:3,1:3)*P;
 Lhat=skew(L);
% Jright13=[eye(3) -Lhat; zeros(3) eye(3) ]*J_right
% J-Jright13
JE=[eye(3) -Lhat; zeros(3) eye(3) ]*J0n;
else
    JE=J0n;
end