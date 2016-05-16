clear all,close all,clc

CurvePoints=load('DataOut');
Uv=load('Uv');
X=CurvePoints(1:4:end,4);
Y=CurvePoints(2:4:end,4);

Pux=Uv(1:4:end,4);
Puy=Uv(2:4:end,4);
Puz=Uv(3:4:end,4);

Curvedata = load('Curve');
data = load('bMt');

Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);

plot(Curvedata(:,1),Curvedata(:,3),'r','Linewidth',2.0)
hold on
plot(Px,Py,'b','Linewidth',2.0)

plot(X,Y,'g:','Linewidth',2.0)
plot(Pux,Puy,'m*','MarkerSize',1.5)


legend('Interpolated Curve','Robot Motion','Original Points','Visual Information')
% A note: The interpolated Curve online is the input trajectory to the robot
% it is exactly the same as that calculated offline

%%       plotting Y versus time


plot(Curvedata(:,3)) % this is the y position in time
hold on
plot(Py,'r') % versus the actual robot position
plot(Uv(:,2),'g')
legend('Curve','Robot','Vision')

figure(2)

plot(Curvedata(:,1)) % this is the y position in time
hold on
plot(Px,'r') % versus the actual robot position
plot(Uv(:,1),'g')
legend('Curve','Robot','Vision')

figure(3)
plot(Uv(:,3))
title('Error between XY by vision and XY from Curve')

%%
steptheta=0.005;
plot(X,Y,'g:','Linewidth',2.0)
hold on
plot(Curvedata(:,1),Curvedata(:,3),'k','LineWidth',2.0);
plot(Pux,Puy,'m*','MarkerSize',1.5)

% From the data I should be able to draw the desired curve and the actual curve
j=1;
for i=1:length(Curvedata)
    plot(X,Y,'g:','Linewidth',2.0)
    plot(Pux,Puy,'m*','MarkerSize',1.5)
    hold on
    plot(Curvedata(:,1),Curvedata(:,3),'k','LineWidth',2.0);
    % Unpack data
    Xt=Curvedata(i,1);
    Yt=Curvedata(i,3);
    theta=Curvedata(i,5);
    bMt=data(j:j+3,:);


    % Draws the current frame
    plot(bMt(1,4),bMt(2,4),'rx');
    drawframe2D(bMt,0.01)

    TangentDirection=[(Xt+steptheta);theta*((Xt+steptheta)-Xt)+Yt]-[Xt;Yt];

   % By theta is not working at the moment
    % Normal direction
    N1=[-TangentDirection(2);TangentDirection(1);0];
    N3=[TangentDirection(2);-TangentDirection(1);0];
    N2=[0;0;1];
    
   
    %         % plot normal one
    plot([Xt,Xt+N1(1)],[Yt,Yt+N1(2)],'g--','LineWidth',1.0)
    %             
    %         % plot normal two 
    plot([Xt,Xt+N3(1)],[Yt,Yt+N3(2)],'g--','LineWidth',1.0)
    j=j+4;
    %pause(0.005)
    pause(0.005)
    hold off

end
%%

S = load('S');
