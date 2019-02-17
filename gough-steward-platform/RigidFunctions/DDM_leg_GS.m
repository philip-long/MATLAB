% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Documents/functions/SYMORO/GS/GS_leg1.ddm




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




%  Direct Calculation of the Joints Accelerations

% Equations:

% Declaration of the function
function QDP=DDM_leg_GS(u)

% Declaration of global input variables
global XX2 XY2 XZ2 YY2 YZ2 ZZ2
global XX3 XY3 XZ3 YY3 YZ3 ZZ3 MX3 MY3 MZ3
global IA3 M3 FZ3  FV3 FS3 CX3 CY3 CZ3
global FX3 FY3 MZ2 MY2 MX2 CX2 CY2 CZ2 IA2 
global FV2 FS2 ZZ1 MY1 MX1 CZ1 IA1 FV1 FS1
global G3

% Declaration of global output variables


t1=u(1);
t2=u(2);
r3=u(3);
QP1=u(4);
QP2=u(5);
QP3=u(6);
GAM1=u(7);
GAM2=u(8);
GAM3=u(9);

% Function description:

	S1=sin(t1);
	C1=cos(t1);
	S2=sin(t2);
	C2=cos(t2);
	WI12=QP1.*S2;
	WI22=C2.*QP1;
	JW12=WI12.*XX2 + WI22.*XY2 + QP2.*XZ2;
	JW22=WI12.*XY2 + WI22.*YY2 + QP2.*YZ2;
	JW32=WI12.*XZ2 + WI22.*YZ2 + QP2.*ZZ2;
	KW12=-(JW22.*QP2) + JW32.*WI22;
	KW22=JW12.*QP2 - JW32.*WI12;
	KW32=JW22.*WI12 - JW12.*WI22;
	WQ12=QP2.*WI22;
	WQ22=-(QP2.*WI12);
	JW13=WI12.*XX3 + QP2.*XY3 - WI22.*XZ3;
	JW23=WI12.*XY3 + QP2.*YY3 - WI22.*YZ3;
	JW33=WI12.*XZ3 + QP2.*YZ3 - WI22.*ZZ3;
	KW13=JW33.*QP2 + JW23.*WI22;
	KW23=-(JW33.*WI12) - JW13.*WI22;
	KW33=-(JW13.*QP2) + JW23.*WI12;
	SW13=QP2.*(-(MX3.*QP2) + MY3.*WI12) + WI22.*(-(MZ3.*WI12) - MX3.*WI22);
	SW23=-(WI12.*(-(MX3.*QP2) + MY3.*WI12)) - WI22.*(MZ3.*QP2 + MY3.*WI22);
	SW33=WI12.*(-(MZ3.*WI12) - MX3.*WI22) - QP2.*(MZ3.*QP2 + MY3.*WI22);
	WQ13=QP2.*QP3;
	WQ23=-(QP3.*WI12);
	LW13=-(r3.*WI12.*WI22) + 2.*WQ13;
	LW23=-(QP2.*r3.*WI22) + 2.*WQ23;
	LW33=-(QP2.^2.*r3) - r3.*WI12.^2;
	JD3=1./(IA3 + M3);
	JU13=JD3.*MY3;
	JU23=-(JD3.*MX3);
	JU63=JD3.*M3;
	GW3=-FZ3 + GAM3 - FV3.*QP3 - SW33 - FS3.*sign(QP3);
	GK113=-(JU13.*MY3) + XX3;
	GK163=-(JU13.*M3) + MY3;
	GK213=-(JU23.*MY3) + XY3;
	GK223=JU23.*MX3 + YY3;
	GK263=-(JU23.*M3) - MX3;
	GK613=MY3 - JU63.*MY3;
	GK623=-MX3 + JU63.*MX3;
	NG13=GK163.*LW33 - LW23.*MZ3;
	NG23=GK263.*LW33 + LW13.*MZ3;
	NG33=LW23.*MX3 - LW13.*MY3;
	NG43=LW13.*M3;
	NG53=LW23.*M3;
	VS13=GW3.*JU13 + NG13;
	VS23=GW3.*JU23 + NG23;
	AP13=CX3 + KW13 + VS13;
	AP23=CY3 + KW23 + VS23;
	AP33=CZ3 + KW33 + NG33;
	AP43=FX3 + NG43 + SW13;
	AP53=FY3 + NG53 + SW23;
	GX113=GK113 + MZ3.*r3;
	GX153=-MZ3 - M3.*r3;
	GX323=GK223 + MZ3.*r3;
	GX333=-(MY3.*r3) + YZ3;
	GX343=MZ3 + M3.*r3;
	TKT113=GX113 - GX153.*r3;
	TKT213=MX3.*r3 - XZ3;
	TKT613=-MZ3 - M3.*r3;
	TKT333=GX323 + GX343.*r3;
	TKT433=MZ3 + M3.*r3;
	MJE112=TKT113 + XX2;
	MJE212=TKT213 + XY2;
	MJE312=GK213 + XZ2;
	MJE512=-GK613 - MZ2;
	MJE612=MY2 + TKT613;
	MJE222=YY2 + ZZ3;
	MJE322=-GX333 + YZ2;
	MJE422=MY3 + MZ2;
	MJE622=-MX2 - MX3;
	MJE332=TKT333 + ZZ2;
	MJE432=-MY2 + TKT433;
	MJE532=-GK623 + MX2;
	VBE12=-AP13 - CX2 - KW12 + AP53.*r3;
	VBE22=AP33 - CY2 - KW22;
	VBE32=-AP23 - CZ2 - KW32 - AP43.*r3;
	JD2=1./(IA2 + MJE332);
	JU12=JD2.*MJE312;
	JU22=JD2.*MJE322;
	JU42=JD2.*MJE432;
	JU52=JD2.*MJE532;
	GW2=GAM2 - FV2.*QP2 + VBE32 - FS2.*sign(QP2);
	GK112=MJE112 - JU12.*MJE312;
	GK122=MJE212 - JU12.*MJE322;
	GK212=MJE212 - JU22.*MJE312;
	GK222=MJE222 - JU22.*MJE322;
	GK412=-(JU42.*MJE312);
	GK422=-(JU42.*MJE322) + MJE422;
	GK512=-(JU52.*MJE312) + MJE512;
	GK522=-(JU52.*MJE322);
	NG12=GK112.*WQ12 + GK122.*WQ22;
	NG22=GK212.*WQ12 + GK222.*WQ22;
	VS12=GW2.*JU12 + NG12;
	VS22=GW2.*JU22 + NG22;
	AP12=-VBE12 + VS12;
	AP22=-VBE22 + VS22;
	GX312=C2.*GK212 + GK112.*S2;
	GX322=C2.*GK222 + GK122.*S2;
	GX412=C2.*GK412 - GK512.*S2;
	GX422=C2.*GK422 - GK522.*S2;
	TKT332=C2.*GX322 + GX312.*S2;
	TKT432=C2.*GX422 + GX412.*S2;
	TKT532=-(C2.*MJE622) - MJE612.*S2;
	MJE331=TKT332 + ZZ1;
	MJE431=-MY1 + TKT432;
	MJE531=MX1 + TKT532;
	VBE31=-(AP22.*C2) - CZ1 - AP12.*S2;
	JD1=1./(IA1 + MJE331);
	JU41=JD1.*MJE431;
	JU51=JD1.*MJE531;
	GW1=GAM1 - FV1.*QP1 + VBE31 - FS1.*sign(QP1);
	VR41=G3.*S1;
	VR51=C1.*G3;
	GU1=JU41.*VR41 + JU51.*VR51;
	QDP1=-GU1 + GW1.*JD1;
	VR12=QDP1.*S2 + WQ12;
	VR22=C2.*QDP1 + WQ22;
	VR42=C2.*VR41;
	VR52=-(S2.*VR41);
	GU2=JU12.*VR12 + JU22.*VR22 + JU42.*VR42 + JU52.*VR52;
	QDP2=-GU2 + GW2.*JD2;
	VR63=LW33 - VR52;
	GU3=JU23.*QDP2 + JU13.*VR12 + JU63.*VR63;
	QDP3=-GU3 + GW3.*JD3;
    QDP=[QDP1 QDP2 QDP3];

% *=*
% Number of operations : 131 '+' or '-', 159 '*' or '/'

