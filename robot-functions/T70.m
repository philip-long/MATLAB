% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)


%    Name of file : D:/Mes documents/KUKA/SymoroFiles/KukalwrAmine.fgm




%      Geometric parameters   


% j     ant   mu    sigma gamma b     alpha d     theta r


% 1     0     1     0     0     0     0     0     t1    r1

%                                     Pi
% 2     1     1     0     0     0     --    0     t2    0
%                                     2
%                                     -Pi
% 3     2     1     0     0     0     ---   0     t3    r3
%                                      2
%                                     -Pi
% 4     3     1     0     0     0     ---   0     t4    0
%                                      2
%                                     Pi
% 5     4     1     0     0     0     --    0     t5    r5
%                                     2
%                                     Pi
% 6     5     1     0     0     0     --    0     t6    0
%                                     2
%                                     -Pi
% 7     6     1     0     0     0     ---   0     t7    r7
%                                      2



%   Transformation Matrix From  R 0  to  R  7 :
% -------------------------------------------
% Equations:

% Declaration of the function
function T0T7=TE0(u)

% Declaration of global input variables
global r1 r3 r5 r7
% Declaration of global output variables


% Function description:
t1=u(1);
t2=u(2);
t3=u(3);
t4=u(4);
t5=u(5);
t6=u(6);
t7=u(7);
T0T7=zeros(4);
	U1T211=cos(t1).*cos(t2);
	U1T212=-(cos(t1).*sin(t2));
	U1T221=cos(t2).*sin(t1);
	U1T222=-(sin(t1).*sin(t2));
	U1T311=U1T211.*cos(t3) - sin(t1).*sin(t3);
	U1T312=-(cos(t3).*sin(t1)) - U1T211.*sin(t3);
	U1T314=r3.*U1T212;
	U1T321=U1T221.*cos(t3) + cos(t1).*sin(t3);
	U1T322=cos(t1).*cos(t3) - U1T221.*sin(t3);
	U1T324=r3.*U1T222;
	U1T331=cos(t3).*sin(t2);
	U1T332=-(sin(t2).*sin(t3));
	U1T334=r1 + r3.*cos(t2);
	U1T411=U1T311.*cos(t4) - U1T212.*sin(t4);
	U1T412=-(U1T212.*cos(t4)) - U1T311.*sin(t4);
	U1T421=U1T321.*cos(t4) - U1T222.*sin(t4);
	U1T422=-(U1T222.*cos(t4)) - U1T321.*sin(t4);
	U1T431=U1T331.*cos(t4) - cos(t2).*sin(t4);
	U1T432=-(cos(t2).*cos(t4)) - U1T331.*sin(t4);
	U1T511=U1T411.*cos(t5) + U1T312.*sin(t5);
	U1T512=U1T312.*cos(t5) - U1T411.*sin(t5);
	U1T514=U1T314 - r5.*U1T412;
	U1T521=U1T421.*cos(t5) + U1T322.*sin(t5);
	U1T522=U1T322.*cos(t5) - U1T421.*sin(t5);
	U1T524=U1T324 - r5.*U1T422;
	U1T531=U1T431.*cos(t5) + U1T332.*sin(t5);
	U1T532=U1T332.*cos(t5) - U1T431.*sin(t5);
	U1T534=U1T334 - r5.*U1T432;
	U1T611=U1T511.*cos(t6) - U1T412.*sin(t6);
	U1T612=-(U1T412.*cos(t6)) - U1T511.*sin(t6);
	U1T621=U1T521.*cos(t6) - U1T422.*sin(t6);
	U1T622=-(U1T422.*cos(t6)) - U1T521.*sin(t6);
	U1T631=U1T531.*cos(t6) - U1T432.*sin(t6);
	U1T632=-(U1T432.*cos(t6)) - U1T531.*sin(t6);
	U1T711=U1T611.*cos(t7) + U1T512.*sin(t7);
	U1T712=U1T512.*cos(t7) - U1T611.*sin(t7);
	U1T714=U1T514 + r7.*U1T612;
	U1T721=U1T621.*cos(t7) + U1T522.*sin(t7);
	U1T722=U1T522.*cos(t7) - U1T621.*sin(t7);
	U1T724=U1T524 + r7.*U1T622;
	U1T731=U1T631.*cos(t7) + U1T532.*sin(t7);
	U1T732=U1T532.*cos(t7) - U1T631.*sin(t7);
	U1T734=U1T534 + r7.*U1T632;
	T0T7(1,1)= U1T711;
	T0T7(2,1)= U1T721;
	T0T7(3,1)= U1T731;
	T0T7(1,2)= U1T712;
	T0T7(2,2)= U1T722;
	T0T7(3,2)= U1T732;
	T0T7(1,3)= U1T612;
	T0T7(2,3)= U1T622;
	T0T7(3,3)= U1T632;
	T0T7(1,4)= U1T714;
	T0T7(2,4)= U1T724;
	T0T7(3,4)= U1T734;
    T0T7(4,:)=[0 0 0 1];

% *=*
% Number of operations : 35 '+' or '-', 71 '*' or '/'
