% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Documents/functions/SYMORO/GS/GS_leg1.inm




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

%                       Inertia matrix
% Equations:

% Declaration of the function
function A_()

% Declaration of global input variables
global t2 MZ3 r3 MX3 MY3 M3 XX2 XX3 XY2 XZ3
global XY3 XZ2 YY2 ZZ3 YZ2 YZ3 YY3 ZZ2 ZZ1 IA1
global IA2 IA3

% Declaration of global output variables
global A

% Function description:

	S2=sin(t2);
	C2=cos(t2);
	PAS113=-(MZ3.*r3);
	PAS123=-(MX3.*r3);
	PAS323=-(MY3.*r3);
	PAS333=-(MZ3.*r3);
	XXP2=-2.*PAS113 + M3.*r3.^2 + XX2 + XX3;
	XYP2=-PAS123 + XY2 - XZ3;
	XZP2=XY3 + XZ2;
	YYP2=YY2 + ZZ3;
	YZP2=-PAS323 + YZ2 - YZ3;
	ZZP2=-2.*PAS333 + M3.*r3.^2 + YY3 + ZZ2;
	AJ312=S2.*XXP2 + C2.*(-PAS123 + XY2 - XZ3);
	AJ322=S2.*XYP2 + C2.*YYP2;
	AJ332=S2.*XZP2 + C2.*YZP2;
	AJA332=AJ322.*C2 + AJ312.*S2;
	ZZP1=AJA332 + ZZ1;
	NC33=MY3.*S2;
	A(1,1)=IA1 + ZZP1;
	A(2,1)=AJ332;
	A(3,1)=NC33;
	A(2,2)=IA2 + ZZP2;
	A(3,2)=-MX3;
	A(3,3)=IA3 + M3;


% *=*
% Number of operations : 22 '+' or '-', 19 '*' or '/'

%  QDP= 
% {QDP1, QDP2, QDP3}

