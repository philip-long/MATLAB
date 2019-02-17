% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Documents/KUKA/SymoroFiles/KukaRed.jpqp




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

%                             JPQP 


% Equations:

% Declaration of the function
function J=JpQp40(u)

% Declaration of global input variables
global r3 Joint2Offset


% Function description:
    t1=u(1);
    t2=u(2)+Joint2Offset;
    t3=u(3);
    t4=u(4);
    QP1=u(5);
    QP2=u(6);
    QP3=u(7);
    QP4=u(8);




	S2=-cos(t2);
	C2=sin(t2);
	S3=sin(t3);
	C3=cos(t3);
	S4=sin(t4);
	C4=cos(t4);
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
	WPJ14=QP4.*WI24 + C4.*WPJ13 - S4.*WPJ22;
	WPJ24=-(QP4.*WI14) - S4.*WPJ13 - C4.*WPJ22;
	WPJ34=WPJ23;
	VPJ14=C4.*VPJ13 - S4.*VSP23;
	VPJ24=-(S4.*VPJ13) - C4.*VSP23;
	VPJ34=VPJ23;
    J=[VPJ14 ;VPJ24; VPJ34; WPJ14; WPJ24; WPJ34];

% *=*
% Number of operations : 17 '+' or '-', 37 '*' or '/'
