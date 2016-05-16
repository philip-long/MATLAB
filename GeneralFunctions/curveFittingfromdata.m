%
clear all clc
bMt=load('DataOut')
Px=[bMt(1:4:end,4)];
Py=[bMt(2:4:end,4)]+(ones(length(Px),1)*0.005);

P=polyfit(Px,Py,2)
plot(Px,Py,'r*')
hold on
plot(Px,polyval(P,Px),'b')

fprintf('p[0]=%f ; p[1]=%f ; p[2]=%f;\n',P(3),P(2),P(1));


%% This is the total cutting time
%
%clear all,clc
P=[-4.9552   -5.4621   -1.0852];
Counter=0;
tfcut=40.2;
Cycletime=0.060;% I think this is 50ms
Xinitial=-0.6638;
Xfinal=-0.440;

dydx=polyder(P);
syms x
dydxsym=poly2sym(dydx,x);
dydx2=(1+(dydxsym*dydxsym))^0.5;
%P2=((int(dydx2,x ,'IgnoreAnalyticConstraints', true)));
P2=((int(dydx2,x, true)));
D=subs(P2,Xfinal)-subs(P2,Xinitial)
P2atXmin=subs(P2,Xinitial);
for t=0:Cycletime:tfcut
    Counter=Counter+1;
    rt=t/tfcut;
    S=rt*D;
    Xt=double((solve(P2-(S+P2atXmin),x)))
    Yt=polyval(P,Xt);
    theta=polyval(polyder(P),Xt);
    
    X_store(Counter)=Xt;
    Y_store(Counter)=Yt;
    theta_store(Counter)=theta;
end

A=[X_store',Y_store',theta_store']
file1=fopen('CurveTrajectory','w');
for i=1:length(A)
    for j=1:3
    fprintf(file1,'\t');fprintf(file1,num2str(A(i,j)));
    end
    fprintf(file1,'\n');
end
fclose(file1);   
%% Test to see if the saved variables are good
Counter=0;
steptheta=0.01;

Xinitial=-0.699;
Xfinal=-0.6;
Xt=X_store(1);
Yt=Y_store(1);
theta=theta_store(1);
steptheta=0.1;
TangentDirection=[steptheta;theta*(steptheta)];
TangentDirection=[TangentDirection;0]/norm([TangentDirection;0]);

N1=[-TangentDirection(2);TangentDirection(1);0];
N3=[TangentDirection(2);-TangentDirection(1);0];
N2=[0;0;1];

P0D=[Xt;Yt;0];

bRt=[N1, TangentDirection, N2];
bMt=[bRt P0D;0 0 0 1];

 plot(Xinitial:0.01:Xfinal,polyval(P,Xinitial:0.01:Xfinal),'r')
hold on
plot(X_store,Y_store,'b')

for t=0:Cycletime:tfcut
    
    Counter=Counter+1;
    bMtm1=bMt;
    bPt_robot=bMtm1(1:3,4);
    bRt_robot=bMtm1(1:3,1:3);
    IndexPoint=floor((t/Cycletime)+1);
    Xt=X_store(IndexPoint);
    Yt=Y_store(IndexPoint);
    theta=theta_store(IndexPoint);
    
    
    
TangentDirection=[steptheta;theta*(steptheta)];
TangentDirection=[TangentDirection;0]/norm([TangentDirection;0]);

N1=[-TangentDirection(2);TangentDirection(1);0];
N3=[TangentDirection(2);-TangentDirection(1);0];
N2=[0;0;1];

P0D=[Xt;Yt;0];

bRt=[N1, TangentDirection, N2];
bMt=[bRt P0D;0 0 0 1];

% Finally Calculate the velocity sent to robot
% assuming perfect integrator

bVt=100.0*(bPt_robot-P0D)

Rdot=bRt_robot*transpose(bRt);
bOmegat=Rot_to_AngleAxis(Rdot);


tVt=transpose(bRt)*bVt;
        
% Plot desired

plot(Xt,Yt,'b*')
%pause(0.1)

bVplot(Counter,:)=bVt;
tVplot(Counter,:)=tVt;
end


plot(bVplot(:,1),'r')
hold on
plot(bVplot(:,2),'b')

plot(tVplot(:,1),'r')
plot(tVplot(:,2),'b')


A=[X_store',Y_store',theta_store']
