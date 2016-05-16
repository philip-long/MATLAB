% This script is just to see if we can do anything
% with the theta issue
Xinitial=-0.9;
Xfinal=-0.5;

X=Xinitial:.050:Xinitial;
nthPolynomial=[-3.15 -5.16 2]
Y=-3.15*(X.^2)-5.16*(X)+2;


tf=10
% Ok so we have this curve in space we want to fix
% an axis on it at each point
Xt=Xinitial;
Yt=Xinitial;
for t=0:0.1:tf
    
        P1=[Xt,Yt];
        rt=(10*((t/(tf))^3))-(15*((t/(tf))^4))+(6*((t/(tf))^5));
        rtdot =(30*t^2)/(tf)^3 - (60*t^3)/(tf)^4 + (30*t^4)/(tf)^5;
       
        Xt=rt*(Xfinal-Xinitial)+Xinitial;
        Xdot=rtdot*(Xfinal-Xinitial);
        Xddot=rtdot*(Xfinal-Xinitial);
        Yt=polyval(nthPolynomial,Xt);
        Ydot=polyval(polyder(nthPolynomial),Xt)*Xdot;
        theta=polyval(polyder(nthPolynomial),Xt)
        %plot(Xt,Yt)
        %hold on
        pause(0.05)
        P2=[Xt,Yt];
        Slope=(P2(2)-P1(2))/(P2(1)-P1(1));
        
        D1=P2-P1
        Dx=[1;0]
        Slope2=(D1(2)-Dx(2))/(D1(1)-Dx(1));
        plot(t,Slope,'r')
        hold on
        plot(t,theta,'b')
        plot(t,Slope2,'g*')
end
P2=[1,0];P1=[2,0]
        D1=P2-P1
        Dx=[1 0]
        Slope2=(D1(2)-Dx(2))/(D1(1)-Dx(1));

% Concl 1 slope and theta are the same
% Concl 2 we need to find theta with respect to
% some reference the x axis for instance


%% Do it with a straight line
Xinitial=-0.9;
Xfinal=-0.5;

X=Xinitial:.050:Xinitial;
nthPolynomial=[-5.16 2]


Xt=Xinitial;
Yt=Xinitial;
for t=0:0.1:tf
    
        P1=[Xt,Yt];
        rt=(10*((t/(tf))^3))-(15*((t/(tf))^4))+(6*((t/(tf))^5));
        rtdot =(30*t^2)/(tf)^3 - (60*t^3)/(tf)^4 + (30*t^4)/(tf)^5;
       
        Xt=rt*(Xfinal-Xinitial)+Xinitial;
        Xdot=rtdot*(Xfinal-Xinitial);
        Xddot=rtdot*(Xfinal-Xinitial);
        Yt=polyval(nthPolynomial,Xt);
        Ydot=polyval(polyder(nthPolynomial),Xt)*Xdot;
        theta=polyval(polyder(nthPolynomial),Xt)
        plot(Xt,Yt)
        hold on
        pause(0.05)
        P2=[Xt,Yt];
        Slope=(P2(2)-P1(2))/(P2(1)-P1(1));
        
        D1=P2-P1
        Dx=[1;0]
        Slope2=(D1(2)-Dx(2))/(D1(1)-Dx(1));
%         plot(t,Slope,'r')
%         hold on
%         plot(t,theta,'b')
%         plot(t,Slope2,'g*')
end


%%

clear all,clc

nthPolynomial=[1 -0.5 2 -3 5 -2]
Xinitial=-2
Xfinal=2
tf=10
Xt=Xinitial;
Yt=4;
for t=0:0.1:tf
    
        P1=[Xt,Yt];
        rt=(10*((t/(tf))^3))-(15*((t/(tf))^4))+(6*((t/(tf))^5));
        rtdot =(30*t^2)/(tf)^3 - (60*t^3)/(tf)^4 + (30*t^4)/(tf)^5;
       
        Xt=rt*(Xfinal-Xinitial)+Xinitial;
        Xdot=rtdot*(Xfinal-Xinitial);
        Xddot=rtdot*(Xfinal-Xinitial);
        Yt=polyval(nthPolynomial,Xt);
        Ydot=polyval(polyder(nthPolynomial),Xt)*Xdot;
        theta=polyval(polyder(nthPolynomial),Xt)
        plot(Xt,Yt,'r*')
        hold on
        pause(0.05)
        P2=[Xt,Yt];
        Slope=(P2(2)-P1(2))/(P2(1)-P1(1));
        
        D1=P2-P1
        Dx=[1;0]
        Slope2=(D1(2)-Dx(2))/(D1(1)-Dx(1));
         plot(t,Slope,'r')
         hold on
        plot(t,theta,'b')
         
end