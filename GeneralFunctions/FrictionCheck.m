syms eta L w tau f
%
th=2*L*w*(eta/(1+eta^2));
tv=2*L*w*(1/(1+eta^2));

syms dv dh
eta=dv/dh

thn=simplify(subs(th))
tvn=simplify(subs(tv))


%%
clear all
syms V H eta Rw
T=(V^2 + H^2)^0.5*(1 + eta^2)%^0.5-Rw
V=Rw*(1/(1+eta^2))
H=Rw*(eta/(1+eta^2))

subs(T)


%% Linear trajectory 

X=[0 1 2 3 4 5 6 7 8 9 10];
Y=[1 2 3 4 5 6 5 4 3 2 1 ];
P=polyfit(X,Y,2);

D=X(end)-X(1);


tf=10.0;
Xtlast=0.0;Ytlast=0.0;
for t=0:0.01:tf
    r=(t/tf)
    Xtlast=Xt;
    Ytlast=Yt
    
    Xt=X(1)+r*D;
    Yt=polyval(P,Xt);
    
    vx=(Xt-Xtlast)/0.01
    vy=(Yt-Ytlast)/0.01
    %plot(Xt,Yt)
    plot(t,vx,'r')
    hold on
    plot(t,vy,'b')
end

