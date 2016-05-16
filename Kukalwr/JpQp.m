% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Mes documents/KUKA/SymoroFiles/KukalwrAmine.jpqp




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

%                             JPQP 


% Equations:

function J=JpQp(u)

% Declaration of global input variables
% Gives the partial acceleration of the task frame assuming the joint
% accleration is zero
global r1 r3 r5 r7 Joint2Offset

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

% Function description:

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
	WI12=QP1.*S2;
	WI22=C2.*QP1;
	WPJ12=QP2.*WI22;
	WPJ22=-(QP2.*WI12);
	DV112=-WI12.^2;
	DV332=-QP2.^2;
	DV122=WI12.*WI22;
	DV232=QP2.*WI22;
	U222=DV112 + DV332;
	U322=DV232 + WPJ12;
	WI13=-(QP2.*S3) + C3.*WI12;
	WI23=-(C3.*QP2) - S3.*WI12;
	W33=QP3 + WI22;
	WPJ13=QP3.*WI23 + C3.*WPJ12;
	WPJ23=-(QP3.*WI13) - S3.*WPJ12;
	VSP13=DV122.*r3;
	VSP23=r3.*U222;
	VSP33=r3.*U322;
	VPJ13=C3.*VSP13 - S3.*VSP33;
	VPJ23=-(S3.*VSP13) - C3.*VSP33;
	WI14=-(S4.*W33) + C4.*WI13;
	WI24=-(C4.*W33) - S4.*WI13;
	W34=QP4 + WI23;
	WPJ14=QP4.*WI24 + C4.*WPJ13 - S4.*WPJ22;
	WPJ24=-(QP4.*WI14) - S4.*WPJ13 - C4.*WPJ22;
	DV114=-WI14.^2;
	DV334=-W34.^2;
	DV124=WI14.*WI24;
	DV234=W34.*WI24;
	U124=DV124 - WPJ23;
	U224=DV114 + DV334;
	U324=DV234 + WPJ14;
	VPJ14=C4.*VPJ13 - S4.*VSP23;
	VPJ24=-(S4.*VPJ13) - C4.*VSP23;
	WI15=S5.*W34 + C5.*WI14;
	WI25=C5.*W34 - S5.*WI14;
	W35=QP5 - WI24;
	WPJ15=QP5.*WI25 + C5.*WPJ14 + S5.*WPJ23;
	WPJ25=-(QP5.*WI15) - S5.*WPJ14 + C5.*WPJ23;
	VSP15=-(r5.*U124) + VPJ14;
	VSP25=-(r5.*U224) + VPJ24;
	VSP35=-(r5.*U324) + VPJ23;
	VPJ15=C5.*VSP15 + S5.*VSP35;
	VPJ25=-(S5.*VSP15) + C5.*VSP35;
	WI16=S6.*W35 + C6.*WI15;
	WI26=C6.*W35 - S6.*WI15;
	W36=QP6 - WI25;
	WPJ16=QP6.*WI26 + C6.*WPJ15 - S6.*WPJ24;
	WPJ26=-(QP6.*WI16) - S6.*WPJ15 - C6.*WPJ24;
	DV116=-WI16.^2;
	DV336=-W36.^2;
	DV126=WI16.*WI26;
	DV236=W36.*WI26;
	U126=DV126 + WPJ25;
	U226=DV116 + DV336;
	U326=DV236 + WPJ16;
	VPJ16=C6.*VPJ15 - S6.*VSP25;
	VPJ26=-(S6.*VPJ15) - C6.*VSP25;
	WI17=-(S7.*W36) + C7.*WI16;
	WI27=-(C7.*W36) - S7.*WI16;
	WPJ17=QP7.*WI27 + C7.*WPJ16 + S7.*WPJ25;
	WPJ27=-(QP7.*WI17) - S7.*WPJ16 + C7.*WPJ25;
	WPJ37=WPJ26;
	VSP17=r7.*U126 + VPJ16;
	VSP27=r7.*U226 + VPJ26;
	VSP37=r7.*U326 - VPJ25;
	VPJ17=C7.*VSP17 - S7.*VSP37;
	VPJ27=-(S7.*VSP17) - C7.*VSP37;
	VPJ37=VSP27;
     J=[VPJ17 ;VPJ27; VPJ37; WPJ17; WPJ27; WPJ37];

% *=*
% Number of operations : 56 '+' or '-', 93 '*' or '/'
