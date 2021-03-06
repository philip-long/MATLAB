% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Documents/functions/SYMORO/GS/GS_leg1.dyn




%      Geometric parameters   


% j     ant   mu    sigma gamma b     alpha d     theta r

%                                     -Pi
% 1     0     1     0     g1    f1    ---   d1    t1    0
%                                      2
%                                     Pi
% 2     1     1     0     0     0     --    0     t2    0
%                                     2
%                                     Pi
% 3     2     1     1     0     0     --    0     0     r3
%                                     2



%              Inertial parameters

% j     XX    XY    XZ    YY    YZ    ZZ    MX    MY    MZ    M     Ia

% 1     XX1   XY1   XZ1   YY1   YZ1   ZZ1   MX1   MY1   MZ1   M1    IA1

% 2     XX2   XY2   XZ2   YY2   YZ2   ZZ2   MX2   MY2   MZ2   M2    IA2

% 3     XX3   XY3   XZ3   YY3   YZ3   ZZ3   MX3   MY3   MZ3   M3    IA3



%  External forces,friction parameters, joint velocities and accelerations

% j      FX     FY     FZ     CX     CY     CZ     FS     FV     QP     QDP

% 1      FX1    FY1    FZ1    CX1    CY1    CZ1    FS1    FV1    QP1    QDP1

% 2      FX2    FY2    FZ2    CX2    CY2    CZ2    FS2    FV2    QP2    QDP2

% 3      FX3    FY3    FZ3    CX3    CY3    CZ3    FS3    FV3    QP3    QDP3

% Base velocity, base accelerations, and gravity

% j     W0    WP0   V0    VP0   G

% 1     0     0     0     0     0

% 2     0     0     0     0     0

% 3     0     0     0     0     G3

%  Dynamic model: Newton Euler method
% Equations:

% Declaration of the function
function GAMMA=IDM_leg_GS(u)

% Declaration of global input variables
global G3 ZZ1 YY2 ZZ2
global XX2 XY2 XZ2 YZ2 MX3 MY3 MZ3
global M3 YY3 ZZ3 XX3 XY3 XZ3 YZ3 FX3 FY3 FZ3
global CX3 CY3 CZ3 CX2 MY2 MZ2 CY2 MX2 CZ2 CZ1
global MY1 MX1 IA1 FV1 FS1 IA2 FV2 FS2 IA3 FV3
global FS3

t1=u(1);
t2=u(2);
r3=u(3);
QP1=u(4);
QP2=u(5);
QP3=u(6);
QDP1=u(7);
QDP2=u(8);
QDP3=u(9);

% switch l
%     case 1
%         g1=0;d1=0;b1=0;
%     case 2
%         g1=0;d1=1.0;b1=0;
%     case 3
%         g1=pi/6;d1= 1.73203810581638;b1=0;
%     case 4
%         g1=pi/3;d1= 2.00082208104569 ;b1=0;
%     case 5
%         g1=pi/2;d1= 1.733 ;b1=0;
%     case 6
%         g1=2*pi/3;d1=0.999977999757995;b1=0;
%     otherwise
%         disp 'defaulting to leg 1'
%         g1=0;d1=0;b1=0;
% end
% Declaration of global output variables


% Function description:

	S1=sin(t1);
	C1=cos(t1);
	S2=sin(t2);
	C2=cos(t2);
	VP11=G3.*S1;
	VP21=C1.*G3;
	No31=QDP1.*ZZ1;
	WI12=QP1.*S2;
	WI22=C2.*QP1;
	WP12=QDP1.*S2 + QP2.*WI22;
	WP22=C2.*QDP1 - QP2.*WI12;
	DV112=-WI12.^2;
	DV222=-WI22.^2;
	DV332=-QP2.^2;
	DV122=WI12.*WI22;
	DV132=QP2.*WI12;
	DV232=QP2.*WI22;
	U122=DV122 - QDP2;
	U132=DV132 + WP22;
	U212=DV122 + QDP2;
	U222=DV112 + DV332;
	U232=DV232 - WP12;
	U312=DV132 - WP22;
	U322=DV232 + WP12;
	VP12=C2.*VP11;
	VP22=-(S2.*VP11);
	PIS12=-YY2 + ZZ2;
	PIS22=XX2 - ZZ2;
	PIS32=-XX2 + YY2;
	No12=DV232.*PIS12 + WP12.*XX2 - U312.*XY2 + U212.*XZ2 + (-DV222 + DV332).*YZ2;
	No22=DV132.*PIS22 + U322.*XY2 + (DV112 - DV332).*XZ2 + WP22.*YY2 - U122.*YZ2;
	No32=DV122.*PIS32 + (-DV112 + DV222).*XY2 - U232.*XZ2 + U132.*YZ2 + QDP2.*ZZ2;
	DV113=-WI12.^2;
	DV223=-QP2.^2;
	DV333=-WI22.^2;
	DV123=QP2.*WI12;
	DV133=-(WI12.*WI22);
	DV233=-(QP2.*WI22);
	U113=DV223 + DV333;
	U123=DV123 + WP22;
	U133=DV133 + QDP2;
	U213=DV123 - WP22;
	U223=DV113 + DV333;
	U233=DV233 - WP12;
	U313=DV133 - QDP2;
	U323=DV233 + WP12;
	U333=DV113 + DV223;
	VSP13=-(r3.*U122) + VP12;
	VSP23=-(r3.*U222) + VP22;
	VSP33=-(r3.*U322) - VP21;
	VP13=2.*QP2.*QP3 + VSP13;
	VP23=VSP33 - 2.*QP3.*WI12;
	VP33=QDP3 - VSP23;
	F13=MX3.*U113 + MY3.*U123 + MZ3.*U133 + M3.*VP13;
	F23=MX3.*U213 + MY3.*U223 + MZ3.*U233 + M3.*VP23;
	F33=MX3.*U313 + MY3.*U323 + MZ3.*U333 + M3.*VP33;
	PIS13=-YY3 + ZZ3;
	PIS23=XX3 - ZZ3;
	PIS33=-XX3 + YY3;
	No13=DV233.*PIS13 + WP12.*XX3 - U313.*XY3 + U213.*XZ3 + (-DV223 + DV333).*YZ3;
	No23=DV133.*PIS23 + U323.*XY3 + (DV113 - DV333).*XZ3 + QDP2.*YY3 - U123.*YZ3;
	No33=DV123.*PIS33 + (-DV113 + DV223).*XY3 - U233.*XZ3 + U133.*YZ3 - WP22.*ZZ3;
	E13=F13 + FX3;
	E23=F23 + FY3;
	E33=F33 + FZ3;
	N13=CX3 + No13 - MZ3.*VP23 + MY3.*VP33;
	N23=CY3 + No23 + MZ3.*VP13 - MX3.*VP33;
	N33=CZ3 + No33 - MY3.*VP13 + MX3.*VP23;
	N12=CX2 + N13 + No12 - E23.*r3 - MY2.*VP21 - MZ2.*VP22;
	N22=CY2 - N33 + No22 + MZ2.*VP12 + MX2.*VP21;
	N32=CZ2 + N23 + No32 + E13.*r3 - MY2.*VP12 + MX2.*VP22;
	N31=CZ1 + C2.*N22 + No31 + N12.*S2 - MY1.*VP11 + MX1.*VP21;
	GAM1=N31 + IA1.*QDP1 + FV1.*QP1 + FS1.*sign(QP1);
	GAM2=N32 + IA2.*QDP2 + FV2.*QP2 + FS2.*sign(QP2);
	GAM3=E33 + IA3.*QDP3 + FV3.*QP3 + FS3.*sign(QP3);
    GAMMA=[GAM1;GAM2;GAM3];

% *=*
% Number of operations : 109 '+' or '-', 99 '*' or '/'
