% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Documents/functions/SYMORO/GS/GS_leg1.fgm




%      Geometric parameters   


% j     ant   mu    sigma gamma b     alpha d     theta r

%                                     -Pi
% 1     0     1     0     g1    b1    ---   d1    t1    0
%                                      2
%                                     Pi
% 2     1     1     0     0     0     --    0     t2    0
%                                     2
%                                     Pi
% 3     2     1     1     0     0     --    0     0     r3
%                                     2



%   Transformation Matrix From  R 0  to  R  3 :
% -------------------------------------------
% Equations:

% Declaration of the function
function T=T03_leg_GS(q,l)

% Declaration of global input variables

global Leg1Origin Leg2Origin Leg3Origin Leg4Origin Leg5Origin Leg6Origin Tw0




t1=q(1);t2=q(2);r3=q(3);
switch l
    case 1
        g1=0;d1=0;b1=0;
    case 2
        g1=atan2(Leg2Origin(2)-Leg1Origin(2),Leg2Origin(1)-Leg1Origin(1));d1=dist2Dpts(Leg1Origin,Leg2Origin);b1=0;
    case 3
        g1=atan2(Leg3Origin(2)-Leg1Origin(2),Leg3Origin(1)-Leg1Origin(1));d1=dist2Dpts(Leg1Origin,Leg3Origin);b1=0;
    case 4
        g1=atan2(Leg4Origin(2)-Leg1Origin(2),Leg4Origin(1)-Leg1Origin(1));d1=dist2Dpts(Leg1Origin,Leg4Origin);b1=0;
    case 5
        g1=atan2(Leg5Origin(2)-Leg1Origin(2),Leg5Origin(1)-Leg1Origin(1));d1=dist2Dpts(Leg1Origin,Leg5Origin);b1=0;
    case 6
        g1=atan2(Leg6Origin(2)-Leg1Origin(2),Leg6Origin(1)-Leg1Origin(1));d1=dist2Dpts(Leg1Origin,Leg6Origin);b1=0;
    otherwise
        disp 'defaulting to leg 1'
        g1=0;d1=0;b1=0;
end



% Function description:

	U1T111=cos(g1).*cos(t1);
	U1T112=-(cos(g1).*sin(t1));
	U1T114=d1.*cos(g1);
	U1T121=cos(t1).*sin(g1);
	U1T122=-(sin(g1).*sin(t1));
	U1T124=d1.*sin(g1);
	U1T211=U1T111.*cos(t2) - sin(g1).*sin(t2);
	U1T212=-(cos(t2).*sin(g1)) - U1T111.*sin(t2);
	U1T221=U1T121.*cos(t2) + cos(g1).*sin(t2);
	U1T222=cos(g1).*cos(t2) - U1T121.*sin(t2);
	U1T231=-(cos(t2).*sin(t1));
	U1T232=sin(t1).*sin(t2);
	U1T314=U1T114 - r3.*U1T212;
	U1T324=U1T124 - r3.*U1T222;
	U1T334=b1 - r3.*U1T232;
	T0T3(1,1)= U1T211;
	T0T3(2,1)= U1T221;
	T0T3(3,1)= U1T231;
	T0T3(1,2)= -U1T112;
	T0T3(2,2)= -U1T122;
	T0T3(3,2)= cos(t1);
	T0T3(1,3)= -U1T212;
	T0T3(2,3)= -U1T222;
	T0T3(3,3)= -U1T232;
	T0T3(1,4)= U1T314;
	T0T3(2,4)= U1T324;
	T0T3(3,4)= U1T334;
    T0T3(4,:)=[0 0 0 1];
    T=Tw0*T0T3;

% *=*
% Number of operations : 7 '+' or '-', 19 '*' or '/'
