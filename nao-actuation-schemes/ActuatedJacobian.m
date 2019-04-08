function Jact=ActuatedJacobian(u)
% This function returns the actuated jacobian matrix for a given joint
% configuration and a given actuation scheme
%
%
%


global Qactuated Qpassive Qcut

Qright=[1,2,3,4,5];
Qleft=[6,7,8,9,10];
Qa=sort(Qactuated);
Qp=sort(Qpassive);
Qc=sort(Qcut);

[J_right,J_left] = Jacobian(u);
[Ga Gac Gp Gpc Gc]=ExtractGaGpGc(u);
G=-Gp\Ga;
G2=-Gc\(Gac+(Gpc*G));

% Vr=Jact qa= J_right qr = J_left *ql
% qr=Upsilonr * qa
% ql=Upsilonl * qa
%
%
% Jact= J_right * UpsilonRight
% Jact= J_left * UpsilonLeft

%Build UpsilonRight



Qa;
Qp;
Qc;

Qright;
Qleft;


% Find Upsilon Right
UpsilonRight=zeros(5,4);
for i=1:5
    if any(ismember(Qa,Qright(i)))
        [r c]=find(Qa==Qright(i)); %find where its equal
        UpsilonRight(i,c)=1;
    elseif any(ismember(Qp,Qright(i)))
        [r c]=find(Qp==Qright(i)); %find where its equal
        UpsilonRight(i,:)=G(c,:);
        
    elseif any(ismember(Qc,Qright(i)))
        find(Qc==Qright(i)); %find where its equal
        UpsilonRight(i,:)=G2;
        
    else
        disp 'non found'
    end
end


UpsilonLeft=zeros(5,4);
% Find Upsilon Left
for i=1:5
    if any(ismember(Qa,Qleft(i)))
        [r c]=find(Qa==Qleft(i)); %find where its equal
        UpsilonLeft(i,c)=1;
    elseif any(ismember(Qp,Qleft(i)))
        [r c]=find(Qp==Qleft(i)); %find where its equal
        UpsilonLeft(i,:)=G(c,:);
        
    elseif any(ismember(Qc,Qleft(i)))
        find(Qc==Qleft(i)); %find where its equal
        UpsilonLeft(i,:)=G2;
        
    else
        disp 'non found'
    end
end

Jactbyl=J_left*UpsilonLeft
Jactbyr=J_right*UpsilonRight

norm(Jactbyl-Jactbyr)
if norm(Jactbyl-Jactbyr)<1e-08
    %disp 'Good Correlation'
    Jact=Jactbyr;
else
    disp 'Jacobian not equal by each chain'
    %norm(Jactbyl-Jactbyr)
    Jact=Jactbyr;
end
    















