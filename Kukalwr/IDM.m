% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Mes documents/KUKA/SymoroFiles/IDM.dyn




%      Geometric parameters   


% j        ant      mu       sigma    gamma    b        alpha    d        theta    r


% 1        0        1        0        0        0        0        0        t1       r1

%                                                       Pi                -Pi
% 2        1        1        0        0        0        --       0        --- + t2 0
%                                                       2                  2
%                                                       -Pi
% 3        2        1        0        0        0        ---      0        t3       r3
%                                                        2
%                                                       -Pi
% 4        3        1        0        0        0        ---      0        t4       0
%                                                        2
%                                                       Pi
% 5        4        1        0        0        0        --       0        t5       r5
%                                                       2
%                                                       Pi
% 6        5        1        0        0        0        --       0        t6       0
%                                                       2
%                                                       -Pi
% 7        6        1        0        0        0        ---      0        t7       r7
%                                                        2



%              Inertial parameters

% j     XX    XY    XZ    YY    YZ    ZZ    MX    MY    MZ    M     Ia

% 1     XX1   XY1   XZ1   YY1   YZ1   ZZ1   MX1   MY1   MZ1   M1    IA1

% 2     XX2   XY2   XZ2   YY2   YZ2   ZZ2   MX2   MY2   MZ2   M2    IA2

% 3     XX3   XY3   XZ3   YY3   YZ3   ZZ3   MX3   MY3   MZ3   M3    IA3

% 4     XX4   XY4   XZ4   YY4   YZ4   ZZ4   MX4   MY4   MZ4   M4    IA4

% 5     XX5   XY5   XZ5   YY5   YZ5   ZZ5   MX5   MY5   MZ5   M5    IA5

% 6     XX6   XY6   XZ6   YY6   YZ6   ZZ6   MX6   MY6   MZ6   M6    IA6

% 7     XX7   XY7   XZ7   YY7   YZ7   ZZ7   MX7   MY7   MZ7   M7    IA7



%  External forces,friction parameters, joint velocities and accelerations

% j      FX     FY     FZ     CX     CY     CZ     FS     FV     QP     QDP

% 1      FX1    FY1    FZ1    CX1    CY1    CZ1    FS1    FV1    QP1    QDP1

% 2      FX2    FY2    FZ2    CX2    CY2    CZ2    FS2    FV2    QP2    QDP2

% 3      FX3    FY3    FZ3    CX3    CY3    CZ3    FS3    FV3    QP3    QDP3

% 4      FX4    FY4    FZ4    CX4    CY4    CZ4    FS4    FV4    QP4    QDP4

% 5      FX5    FY5    FZ5    CX5    CY5    CZ5    FS5    FV5    QP5    QDP5

% 6      FX6    FY6    FZ6    CX6    CY6    CZ6    FS6    FV6    QP6    QDP6

% 7      FX7    FY7    FZ7    CX7    CY7    CZ7    FS7    FV7    QP7    QDP7

% Base velocity, base accelerations, and gravity

% j     W0    WP0   V0    VP0   G

% 1     0     0     0     0     0

% 2     0     0     0     0     0

% 3     0     0     0     0     G3

%  Dynamic model: Newton Euler method
% Equations:

% Declaration of the function
function GAM=IDM(u)


global r1 r3 r5  Joint2Offset r7_B
% Declaration of global Dynamic variables
global	XX1	XX2	XX3	XX4	XX5	XX6	XX7
global	XY1	XY2	XY3	XY4	XY5	XY6	XY7
global	XZ1	XZ2	XZ3	XZ4	XZ5	XZ6	XZ7
global	YY1	YY2	YY3	YY4	YY5	YY6	YY7
global	YZ1	YZ2	YZ3	YZ4	YZ5	YZ6	YZ7
global	ZZ1	ZZ2	ZZ3	ZZ4	ZZ5	ZZ6	ZZ7
global	MX1	MX2	MX3	MX4	MX5	MX6	MX7
global	MY1	MY2	MY3	MY4	MY5	MY6	MY7
global	MZ1	MZ2	MZ3	MZ4	MZ5	MZ6	MZ7
global	M1	M2	M3	M4	M5	M6	M7
global	IA1	IA2	IA3	IA4	IA5	IA6	IA7
global	FV1	FV2	FV3	FV4	FV5	FV6	FV7
global	FS1	FS2	FS3	FS4	FS5	FS6	FS7
global	FX1	FX2	FX3	FX4	FX5	FX6	FX7
global	FY1	FY2	FY3	FY4	FY5	FY6	FY7
global	FZ1	FZ2	FZ3	FZ4	FZ5	FZ6	FZ7
global	CX1	CX2	CX3	CX4	CX5	CX6	CX7
global	CY1	CY2	CY3	CY4	CY5	CY6	CY7
global	CZ1	CZ2	CZ3	CZ4	CZ5	CZ6	CZ7

global G3% Function description:
%r7=0.078;

r7=r7_B; % r7 is link origin and not tool offset

t1=u(1);
t2=u(2)+Joint2Offset;
t3=u(3);
t4=u(4);
t5=u(5);
t6=u(6);
t7=u(7);
QP1=u(8);
QP2=u(9);
QP3=u(10);
QP4=u(11);
QP5=u(12);
QP6=u(13);
QP7=u(14);
QDP1=u(15);
QDP2=u(16);
QDP3=u(17);
QDP4=u(18);
QDP5=u(19);
QDP6=u(20);
QDP7=u(21);

	S2=-cos(t2);
	C2=sin(t2);
	S3=sin(t3);
	C3=cos(t3);
	S4=sin(t4);
	C4=cos(t4);
	S5=sin(t5);
	C5=cos(t5);
	S6=sin(t6);
	C6=cos(t6);
	S7=sin(t7);
	C7=cos(t7);
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
	VP12=-(G3.*S2);
	VP22=-(C2.*G3);
	PIS12=-YY2 + ZZ2;
	PIS22=XX2 - ZZ2;
	PIS32=-XX2 + YY2;
	No12=DV232.*PIS12 + WP12.*XX2 - U312.*XY2 + U212.*XZ2 + (-DV222 + DV332).*YZ2;
	No22=DV132.*PIS22 + U322.*XY2 + (DV112 - DV332).*XZ2 + WP22.*YY2 - U122.*YZ2;
	No32=DV122.*PIS32 + (-DV112 + DV222).*XY2 - U232.*XZ2 + U132.*YZ2 + QDP2.*ZZ2;
	WI13=-(QP2.*S3) + C3.*WI12;
	WI23=-(C3.*QP2) - S3.*WI12;
	W33=QP3 + WI22;
	WP13=-(QDP2.*S3) + QP3.*WI23 + C3.*WP12;
	WP23=-(C3.*QDP2) - QP3.*WI13 - S3.*WP12;
	WP33=QDP3 + WP22;
	DV113=-WI13.^2;
	DV223=-WI23.^2;
	DV333=-W33.^2;
	DV123=WI13.*WI23;
	DV133=W33.*WI13;
	DV233=W33.*WI23;
	U113=DV223 + DV333;
	U123=DV123 - WP33;
	U133=DV133 + WP23;
	U213=DV123 + WP33;
	U223=DV113 + DV333;
	U233=DV233 - WP13;
	U313=DV133 - WP23;
	U323=DV233 + WP13;
	VSP13=r3.*U122 + VP12;
	VSP23=r3.*U222 + VP22;
	VSP33=r3.*U322;
	VP13=C3.*VSP13 - S3.*VSP33;
	VP23=-(S3.*VSP13) - C3.*VSP33;
	F13=MX3.*U113 + MY3.*U123 + MZ3.*U133 + M3.*VP13;
	F23=MX3.*U213 + MY3.*U223 + MZ3.*U233 + M3.*VP23;
	PIS13=-YY3 + ZZ3;
	PIS23=XX3 - ZZ3;
	PIS33=-XX3 + YY3;
	No13=DV233.*PIS13 + WP13.*XX3 - U313.*XY3 + U213.*XZ3 + (-DV223 + DV333).*YZ3;
	No23=DV133.*PIS23 + U323.*XY3 + (DV113 - DV333).*XZ3 + WP23.*YY3 - U123.*YZ3;
	No33=DV123.*PIS33 + (-DV113 + DV223).*XY3 - U233.*XZ3 + U133.*YZ3 + WP33.*ZZ3;
	WI14=-(S4.*W33) + C4.*WI13;
	WI24=-(C4.*W33) - S4.*WI13;
	W34=QP4 + WI23;
	WP14=QP4.*WI24 + C4.*WP13 - S4.*WP33;
	WP24=-(QP4.*WI14) - S4.*WP13 - C4.*WP33;
	WP34=QDP4 + WP23;
	DV114=-WI14.^2;
	DV224=-WI24.^2;
	DV334=-W34.^2;
	DV124=WI14.*WI24;
	DV134=W34.*WI14;
	DV234=W34.*WI24;
	U114=DV224 + DV334;
	U124=DV124 - WP34;
	U134=DV134 + WP24;
	U214=DV124 + WP34;
	U224=DV114 + DV334;
	U234=DV234 - WP14;
	U314=DV134 - WP24;
	U324=DV234 + WP14;
	U334=DV114 + DV224;
	VP14=C4.*VP13 - S4.*VSP23;
	VP24=-(S4.*VP13) - C4.*VSP23;
	F14=MX4.*U114 + MY4.*U124 + MZ4.*U134 + M4.*VP14;
	F24=MX4.*U214 + MY4.*U224 + MZ4.*U234 + M4.*VP24;
	F34=MX4.*U314 + MY4.*U324 + MZ4.*U334 + M4.*VP23;
	PIS14=-YY4 + ZZ4;
	PIS24=XX4 - ZZ4;
	PIS34=-XX4 + YY4;
	No14=DV234.*PIS14 + WP14.*XX4 - U314.*XY4 + U214.*XZ4 + (-DV224 + DV334).*YZ4;
	No24=DV134.*PIS24 + U324.*XY4 + (DV114 - DV334).*XZ4 + WP24.*YY4 - U124.*YZ4;
	No34=DV124.*PIS34 + (-DV114 + DV224).*XY4 - U234.*XZ4 + U134.*YZ4 + WP34.*ZZ4;
	WI15=S5.*W34 + C5.*WI14;
	WI25=C5.*W34 - S5.*WI14;
	W35=QP5 - WI24;
	WP15=QP5.*WI25 + C5.*WP14 + S5.*WP34;
	WP25=-(QP5.*WI15) - S5.*WP14 + C5.*WP34;
	WP35=QDP5 - WP24;
	DV115=-WI15.^2;
	DV225=-WI25.^2;
	DV335=-W35.^2;
	DV125=WI15.*WI25;
	DV135=W35.*WI15;
	DV235=W35.*WI25;
	U115=DV225 + DV335;
	U125=DV125 - WP35;
	U135=DV135 + WP25;
	U215=DV125 + WP35;
	U225=DV115 + DV335;
	U235=DV235 - WP15;
	U315=DV135 - WP25;
	U325=DV235 + WP15;
	U335=DV115 + DV225;
	VSP15=-(r5.*U124) + VP14;
	VSP25=-(r5.*U224) + VP24;
	VSP35=-(r5.*U324) + VP23;
	VP15=C5.*VSP15 + S5.*VSP35;
	VP25=-(S5.*VSP15) + C5.*VSP35;
	F15=MX5.*U115 + MY5.*U125 + MZ5.*U135 + M5.*VP15;
	F25=MX5.*U215 + MY5.*U225 + MZ5.*U235 + M5.*VP25;
	F35=MX5.*U315 + MY5.*U325 + MZ5.*U335 - M5.*VSP25;
	PIS15=-YY5 + ZZ5;
	PIS25=XX5 - ZZ5;
	PIS35=-XX5 + YY5;
	No15=DV235.*PIS15 + WP15.*XX5 - U315.*XY5 + U215.*XZ5 + (-DV225 + DV335).*YZ5;
	No25=DV135.*PIS25 + U325.*XY5 + (DV115 - DV335).*XZ5 + WP25.*YY5 - U125.*YZ5;
	No35=DV125.*PIS35 + (-DV115 + DV225).*XY5 - U235.*XZ5 + U135.*YZ5 + WP35.*ZZ5;
	WI16=S6.*W35 + C6.*WI15;
	WI26=C6.*W35 - S6.*WI15;
	W36=QP6 - WI25;
	WP16=QP6.*WI26 + C6.*WP15 + S6.*WP35;
	WP26=-(QP6.*WI16) - S6.*WP15 + C6.*WP35;
	WP36=QDP6 - WP25;
	DV116=-WI16.^2;
	DV226=-WI26.^2;
	DV336=-W36.^2;
	DV126=WI16.*WI26;
	DV136=W36.*WI16;
	DV236=W36.*WI26;
	U116=DV226 + DV336;
	U126=DV126 - WP36;
	U136=DV136 + WP26;
	U216=DV126 + WP36;
	U226=DV116 + DV336;
	U236=DV236 - WP16;
	U316=DV136 - WP26;
	U326=DV236 + WP16;
	U336=DV116 + DV226;
	VP16=C6.*VP15 - S6.*VSP25;
	VP26=-(S6.*VP15) - C6.*VSP25;
	F16=MX6.*U116 + MY6.*U126 + MZ6.*U136 + M6.*VP16;
	F26=MX6.*U216 + MY6.*U226 + MZ6.*U236 + M6.*VP26;
	F36=MX6.*U316 + MY6.*U326 + MZ6.*U336 - M6.*VP25;
	PIS16=-YY6 + ZZ6;
	PIS26=XX6 - ZZ6;
	PIS36=-XX6 + YY6;
	No16=DV236.*PIS16 + WP16.*XX6 - U316.*XY6 + U216.*XZ6 + (-DV226 + DV336).*YZ6;
	No26=DV136.*PIS26 + U326.*XY6 + (DV116 - DV336).*XZ6 + WP26.*YY6 - U126.*YZ6;
	No36=DV126.*PIS36 + (-DV116 + DV226).*XY6 - U236.*XZ6 + U136.*YZ6 + WP36.*ZZ6;
	WI17=-(S7.*W36) + C7.*WI16;
	WI27=-(C7.*W36) - S7.*WI16;
	W37=QP7 + WI26;
	WP17=QP7.*WI27 + C7.*WP16 - S7.*WP36;
	WP27=-(QP7.*WI17) - S7.*WP16 - C7.*WP36;
	WP37=QDP7 + WP26;
	DV117=-WI17.^2;
	DV227=-WI27.^2;
	DV337=-W37.^2;
	DV127=WI17.*WI27;
	DV137=W37.*WI17;
	DV237=W37.*WI27;
	U117=DV227 + DV337;
	U127=DV127 - WP37;
	U137=DV137 + WP27;
	U217=DV127 + WP37;
	U227=DV117 + DV337;
	U237=DV237 - WP17;
	U317=DV137 - WP27;
	U327=DV237 + WP17;
	U337=DV117 + DV227;
	VSP17=r7.*U126 + VP16;
	VSP27=r7.*U226 + VP26;
	VSP37=r7.*U326 - VP25;
	VP17=C7.*VSP17 - S7.*VSP37;
	VP27=-(S7.*VSP17) - C7.*VSP37;
	F17=MX7.*U117 + MY7.*U127 + MZ7.*U137 + M7.*VP17;
	F27=MX7.*U217 + MY7.*U227 + MZ7.*U237 + M7.*VP27;
	F37=MX7.*U317 + MY7.*U327 + MZ7.*U337 + M7.*VSP27;
	PIS17=-YY7 + ZZ7;
	PIS27=XX7 - ZZ7;
	PIS37=-XX7 + YY7;
	No17=DV237.*PIS17 + WP17.*XX7 - U317.*XY7 + U217.*XZ7 + (-DV227 + DV337).*YZ7;
	No27=DV137.*PIS27 + U327.*XY7 + (DV117 - DV337).*XZ7 + WP27.*YY7 - U127.*YZ7;
	No37=DV127.*PIS37 + (-DV117 + DV227).*XY7 - U237.*XZ7 + U137.*YZ7 + WP37.*ZZ7;
	E17=F17 + FX7;
	E27=F27 + FY7;
	E37=F37 + FZ7;
	N17=CX7 + No17 - MZ7.*VP27 + MY7.*VSP27;
	N27=CY7 + No27 + MZ7.*VP17 - MX7.*VSP27;
	N37=CZ7 + No37 - MY7.*VP17 + MX7.*VP27;
	FDI17=C7.*E17 - E27.*S7;
	FDI37=-(C7.*E27) - E17.*S7;
	E16=F16 + FDI17 + FX6;
	E26=E37 + F26 + FY6;
	E36=F36 + FDI37 + FZ6;
	N16=CX6 + C7.*N17 + No16 + FDI37.*r7 - N27.*S7 - MY6.*VP25 - MZ6.*VP26;
	N26=CY6 + N37 + No26 + MZ6.*VP16 + MX6.*VP25;
	N36=CZ6 - C7.*N27 + No36 - FDI17.*r7 - N17.*S7 - MY6.*VP16 + MX6.*VP26;
	FDI16=C6.*E16 - E26.*S6;
	FDI36=C6.*E26 + E16.*S6;
	E15=F15 + FDI16 + FX5;
	E25=-E36 + F25 + FY5;
	E35=F35 + FDI36 + FZ5;
	N15=CX5 + C6.*N16 + No15 - N26.*S6 - MZ5.*VP25 - MY5.*VSP25;
	N25=CY5 - N36 + No25 + MZ5.*VP15 + MX5.*VSP25;
	N35=CZ5 + C6.*N26 + No35 + N16.*S6 - MY5.*VP15 + MX5.*VP25;
	FDI15=C5.*E15 - E25.*S5;
	FDI35=C5.*E25 + E15.*S5;
	E14=F14 + FDI15 + FX4;
	E24=-E35 + F24 + FY4;
	E34=F34 + FDI35 + FZ4;
	N14=CX4 + C5.*N15 + No14 - FDI35.*r5 - N25.*S5 + MY4.*VP23 - MZ4.*VP24;
	N24=CY4 - N35 + No24 + MZ4.*VP14 - MX4.*VP23;
	N34=CZ4 + C5.*N25 + No34 + FDI15.*r5 + N15.*S5 - MY4.*VP14 + MX4.*VP24;
	FDI14=C4.*E14 - E24.*S4;
	E13=F13 + FDI14 + FX3;
	E23=E34 + F23 + FY3;
	N13=CX3 + C4.*N14 + No13 - N24.*S4 - MZ3.*VP23 + MY3.*VSP23;
	N23=CY3 + N34 + No23 + MZ3.*VP13 - MX3.*VSP23;
	N33=CZ3 - C4.*N24 + No33 - N14.*S4 - MY3.*VP13 + MX3.*VP23;
	FDI13=C3.*E13 - E23.*S3;
	FDI33=-(C3.*E23) - E13.*S3;
	N12=CX2 + C3.*N13 + No12 + FDI33.*r3 - N23.*S3 - MZ2.*VP22;
	N22=CY2 + N33 + No22 + MZ2.*VP12;
	N32=CZ2 - C3.*N23 + No32 - FDI13.*r3 - N13.*S3 - MY2.*VP12 + MX2.*VP22;
	N31=CZ1 + C2.*N22 + No31 + N12.*S2;
	GAM1=N31 + IA1.*QDP1 + FV1.*QP1 + FS1.*sign(QP1);
	GAM2=N32 + IA2.*QDP2 + FV2.*QP2 + FS2.*sign(QP2);
	GAM3=N33 + IA3.*QDP3 + FV3.*QP3 + FS3.*sign(QP3);
	GAM4=N34 + IA4.*QDP4 + FV4.*QP4 + FS4.*sign(QP4);
	GAM5=N35 + IA5.*QDP5 + FV5.*QP5 + FS5.*sign(QP5);
	GAM6=N36 + IA6.*QDP6 + FV6.*QP6 + FS6.*sign(QP6);
	GAM7=N37 + IA7.*QDP7 + FV7.*QP7 + FS7.*sign(QP7);
    GAM=[GAM1 GAM2 GAM3 GAM4 GAM5 GAM6 GAM7];

% *=*
% Number of operations : 402 '+' or '-', 371 '*' or '/'
