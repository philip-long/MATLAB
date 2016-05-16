% Testing if image control law converges
clear all,clc
Params
global qinit qinitC Sdesired
q1=qinit;
q3=qinitC;
CycleTime=0.1;
Counter=0;

plotseg(Sdesired)
lambda=10;
 
for t=0:CycleTime:20
    
    J=J70_B(q3);
    Tr=T70_B(q3);
    S=SsegfromQ([q1;q3]);
    plotseg(S)
    pause(0.1)
    L=LsegfromQ([q1;q3]);
    DeltaS=S-Sdesired;
    % This generates the velocity of the camera in
    % the camera frame
    Vc=pinv(L)*DeltaS;
    %cond(L*L')
    V=[Tr(1:3,1:3)  zeros(3); zeros(3) Tr(1:3,1:3) ]*Vc;
    V=-lambda.*V;
    dq=pinv(J)*V;
    
    q3=q3+(dq*CycleTime);
    t;
    Counter=Counter+1;
    T(Counter)=t;
    norm(DeltaS)
    DS(:,Counter)=DeltaS;
    
end

figure(2)
plot(T,DS(1:4,:))
legend('x','y','L','theta')



%%

global CameraParameters


CameraParameters=[560; 560; 0;  374.4020879	;  180.2622981 ];

u1=225;u2=250;v1=5;v2=75;

uc=(u1+u2)/2
vc=(v1+v2)/2
l=((u1-u2)^2 +((v1-v2)^2))^0.5
theta=atan((v1-v2)/(u1-u2))


u1=uc+((l/2)*cos(theta))
u2=uc-((l/2)*cos(theta))
v1=vc+((l/2)*sin(theta))
v2=vc-((l/2)*sin(theta))

fu=212;
fv=560;
fs=0;
f=50;
Z1=50;
Z2=50;
u0=12;
v0=55;


J= [  uc*vc/fv+l*l*cos(theta)*sin(theta)/(4*fv)                        -(fu+uc*uc/fu+l*l*cos(theta)*cos(theta)/(4*fu))                 uc/f;
      fv+vc*vc/fv+l*l*sin(theta)*sin(theta)/(4*fv)                     -uc*vc/fu-l*l*cos(theta)*sin(theta)/(4*fu)                      vc/f;
      l*(uc*cos(theta)*sin(theta)+vc*(1+sin(theta)*sin(theta)))/fv     -l*(vc*cos(theta)*sin(theta)+uc*(1+cos(theta)*cos(theta)))/fv   l/f; ]

  
  
L =[ - fu/(2*Z1) - fu/(2*Z2),                      - fs/(2*Z1) - fs/(2*Z2),                                                                                                                                                 (uc - u0 + (l*cos(theta))/2)/(2*Z1) - (u0 - uc + (l*cos(theta))/2)/(2*Z2),                                                                                                            fs + ((sin(2*theta)*l^2)/8 + u0*v0 - u0*vc - uc*v0 + uc*vc)/fv,                                                                                                     ((fs*sin(2*theta)*l^2)/8 + fs*u0*v0 - fs*u0*vc - fs*uc*v0 + fs*uc*vc)/(fu*fv) - ((l^2*cos(theta)^2)/4 - 2*u0*uc + fu^2 + u0^2 + uc^2)/fu,                                                      (fs*(u0 - uc))/fu - ((v0 - vc)*(fs^2 + fu^2))/(fu*fv)
     0,                                           - fv/(2*Z1) - fv/(2*Z2),                                                                                                                                                 (vc - v0 + (l*sin(theta))/2)/(2*Z1) - (v0 - vc + (l*sin(theta))/2)/(2*Z2),                                                                                                                    fv + ((l^2*sin(theta)^2)/4 - 2*v0*vc + v0^2 + vc^2)/fv,                                                                                                               (fs*v0^2 + fs*vc^2 - 2*fs*v0*vc + (fs*l^2*sin(theta)^2)/4)/(fu*fv) - ((sin(2*theta)*l^2)/8 + u0*v0 - u0*vc - uc*v0 + uc*vc)/fu,                                                                         (fv*u0 - fv*uc - fs*v0 + fs*vc)/fu
(fu*l*cos(theta)*(Z1 - Z2))/(Z1*Z2*(l^2)^(1/2)),     (l*(Z1 - Z2)*(fs*cos(theta) + fv*sin(theta)))/(Z1*Z2*(l^2)^(1/2)), (l*(l - 2*u0*cos(theta) + 2*uc*cos(theta) - 2*v0*sin(theta) + 2*vc*sin(theta)))/(2*Z1*(l^2)^(1/2)) + (l*(Z1*l + 2*Z1*u0*cos(theta) - 2*Z1*uc*cos(theta) + 2*Z1*v0*sin(theta) - 2*Z1*vc*sin(theta)))/(2*Z1*Z2*(l^2)^(1/2)), -(2*v0*(l^2)^(1/2) - 2*vc*(l^2)^(1/2) + u0*sin(2*theta)*(l^2)^(1/2) - uc*sin(2*theta)*(l^2)^(1/2) + 2*v0*sin(theta)^2*(l^2)^(1/2) - 2*vc*sin(theta)^2*(l^2)^(1/2))/(2*fv), ((l^2)^(1/2)*(3*u0 - 3*uc + u0*cos(2*theta) - uc*cos(2*theta) + v0*sin(2*theta) - vc*sin(2*theta)))/(2*fu) - ((l^2)^(1/2)*(3*fs*v0 - 3*fs*vc - fs*v0*cos(2*theta) + fs*vc*cos(2*theta) + fs*u0*sin(2*theta) - fs*uc*sin(2*theta)))/(2*fu*fv), ((l^2)^(1/2)*(sin(2*theta)*fs^2 - 2*cos(2*theta)*fs*fv + sin(2*theta)*fu^2 - sin(2*theta)*fv^2))/(2*fu*fv)
-(fu*sin(theta)*(Z1 - Z2))/(Z1*Z2*l),             ((Z1 - Z2)*(fv*cos(theta) - fs*sin(theta)))/(Z1*Z2*l),                                                                                                                                     ((Z1 - Z2)*(v0*cos(theta) - vc*cos(theta) - u0*sin(theta) + uc*sin(theta)))/(Z1*Z2*l),                                                                                        (u0*sin(theta)^2 - uc*sin(theta)^2 - (v0*sin(2*theta))/2 + (vc*sin(2*theta))/2)/fv,                                           (v0 - vc - (u0*sin(2*theta))/2 + (uc*sin(2*theta))/2 - v0*sin(theta)^2 + vc*sin(theta)^2)/fu + (fs*u0*sin(theta)^2 - fs*uc*sin(theta)^2 - (fs*v0*sin(2*theta))/2 + (fs*vc*sin(2*theta))/2)/(fu*fv),        (fv*(sin(theta)^2 - 1))/fu - (fs^2*sin(theta)^2 + fu^2*sin(theta)^2)/(fu*fv) + (fs*sin(2*theta))/fu]



