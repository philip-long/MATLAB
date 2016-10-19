close all,clear all,clc

syms alpha ra rb l1 l2 l3 x y rc rd
syms rc rd l4 l5 l6 
xa=ra*cos(alpha);
ya=-ra*sin(alpha);
xb=rb*cos(alpha);
yb=rb*sin(alpha);


xc=rc*cos(alpha);
yc=rc*sin(alpha);
xd=rd*cos(alpha);
yd=-rd*sin(alpha);

CA_l3_e2=((xb-xa)^2) + ((yb-ya)^2 ) - l3^2;
rb=simplify(solve(CA_l3_e2,rb));
xb=simplify(subs(xb));
yb=simplify(subs(yb));

CC_l6_e1=((xd-xc)^2) + ((yd-yc)^2 ) - l6^2;
rd=simplify(solve(CC_l6_e1,rd));
xd=simplify(subs(xd));
yd=simplify(subs(yd));

% Next find E which is intersection
% of two circles one at A of radius l1
% and a second at B of radius l2

CA_l1=((x-xa)*(x-xa) ) + ((y-ya)*(y-ya) ) - l1*l1;
CC_l4=((x-xc)*(x-xc) ) + ((y-yc)*(y-yc) ) - l4*l4;

for i=1:2
    CB_l2=((x-xb(i))*(x-xb(i)) ) + ((y-yb(i))*(y-yb(i)) ) - l2*l2;    
    diff_circles=CA_l1-CB_l2;
    ys=solve(diff_circles,y);
    xs=solve(subs(CA_l1,y,ys),x);
    ys=subs(ys,x,xs);
    if(i==1)
        xe1=xs; ye1=ys;
    else
        xe2=xs; ye2=ys;
    end        
    
    CD_l5=((x-xd(i))*(x-xd(i)) ) + ((y-yd(i))*(y-yd(i)) ) - l4*l4;
    diff_circles_cd=CC_l4-CD_l5;
    ys_cd=solve(diff_circles_cd,y);
    xs_cd=solve(subs(CA_l1,y,ys_cd),x);
    ys_cd=subs(ys_cd,x,xs_cd);
    if(i==1)
        xe1_cd=xs_cd; ye1_cd=ys_cd;
    else
        xe2_cd=xs_cd; ye2_cd=ys_cd;
    end 
    
end




%% Now try to simulate stuff
alpha=pi/18;
l1=0.7;
l2=0.3;
l3=0.8;
l4=0.65;
l5=0.24;
l6=0.86;
xa=subs(xa); xb=subs(xb); xc=subs(xc); xd=subs(xd);
ya=subs(ya); yb=subs(yb); yc=subs(yc); yd=subs(yd);
xe1=simplify(subs(xe1)); ye1=simplify(subs(ye1));
xe1_cd=simplify(subs(xe1_cd)); ye1_cd=simplify(subs(ye1_cd));

if (abs(l1-l2)<l3 && l3<(l1+l2))
    disp('Condition Satisfied')
else
    disp('Error')
    pause()
end
if (abs(l4-l5)<l6 && l6<(l4+l5))
    disp('Condition Satisfied')
else
    disp('Error')
    pause()
end

ra=0.1;
rc=ra;

plot(0,0,'ro')
hold on
plot([0 2*cos(alpha)],[0 2*sin(alpha)],'k:')
plot([0 2*cos(alpha)],[0 -2*sin(alpha)],'k:')

Pa=[subs(xa),subs(ya)];
Pb=[subs(xb),subs(yb)];
Pe=[subs(xe1),subs(ye1)];
Pc=[subs(xc),subs(yc)];
Pd=[subs(xd),subs(yd)];
Pe_cd=[subs(xe1_cd),subs(ye1_cd)];
%Pe=[subs(xe1),subs(ye1)];

plot(Pa(1),Pa(2),'bo')
plot(Pb(:,1),Pb(:,2),'bo')
plot(Pe(:,1),Pe(:,2),'go')

plot(Pc(1),Pc(2),'ko')
plot(Pd(:,1),Pd(:,2),'ko')
plot(Pe_cd(:,1),Pe_cd(:,2),'go')


