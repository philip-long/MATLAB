% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)
% 
% 
%    Name of file : D:/Mes documents/NAO/Nao SYMORO/4DOF/NaoCL.ckel
% 
% 
% 
% 
%      Geometric parameters   
% 
% 
% j       ant     mu      sigma   gamma   b       alpha   d       theta   r
% 
%                                                 -Pi
% 1       0       1       0       0       b1      ---     d1      t1      r1
%                                                  2
%                                                 Pi              Pi
% 2       1       1       0       0       0       --      0       -- + t2 0
%                                                 2               2
%                                                 Pi
% 3       2       1       0       0       0       --      d3      t3      r3
%                                                 2
%                                                 -Pi
% 4       3       1       0       0       0       ---     0       t4      0
%                                                  2
%                                                 Pi
% 5       4       0       0       0       0       --      0       t5      r5
%                                                 2
%                                                 -Pi
% 6       0       0       0       0       b6      ---     d6      t6      r6
%                                                  2
%                                                 Pi              Pi
% 7       6       0       0       0       0       --      0       -- + t7 0
%                                                 2               2
%                                                 Pi
% 8       7       0       0       0       0       --      d8      t8      r8
%                                                 2
%                                                 -Pi
% 9       8       0       0       0       0       ---     0       t9      0
%                                                  2
%                                                 Pi
% 10      9       0       0       0       0       --      0       t10     r10
%                                                 2
% 
% 11      5       0       2       g11     b11     alpha11 d11     t11     r11
% 
% 
%    Constraint kinematic equations of loops 

% 
%             --------------------------------------
% 
%             Constraint velocity equations of loops  
% 
% 
%          WA  | WP   |   0         QPa
%          ------------------  *    QPp      = 0
%          WAC | WPC  |  WC         QPc 


function [J]=VelocityConstraints(u)


%% Declaration of global input variables

global q_upper q_lower...
    d3 r3...    
    r5...
    g6 r6...
    d8 r8...
    r10...
    b11 g11 alpha11 d11 r11 t11

%Actuated
t1=u(1);
t2=u(2);
t3=u(3);
t4=u(4);

%Passive
t5=u(5);
t6=u(6);
t7=u(7);
t8=u(8);
t9=u(9);

%Cut
t10=u(10);
WA=zeros( 5 , 4 );

WA(1,1) = r3*cos(alpha11)*cos(t2)*cos(t3)*cos(g11 + t5)*sin(t11) + r11*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(t11) + b11*cos(alpha11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(t11) + r5*cos(alpha11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(t11) - r11*cos(t11)*cos(t4)*sin(alpha11)*sin(t2) - d11*cos(alpha11)*cos(t4)*sin(t11)*sin(t2) - d3*cos(alpha11)*cos(t3)*cos(g11 + t5)*sin(t11)*sin(t2) + b11*cos(t11)*cos(t2)*cos(g11 + t5)*sin(t3) + r5*cos(t11)*cos(t2)*cos(g11 + t5)*sin(t3) + r11*cos(alpha11)*cos(t11)*cos(t2)*cos(g11 + t5)*sin(t3) + r3*cos(t11)*cos(t2)*cos(t4)*cos(g11 + t5)*sin(t3) - d11*cos(t2)*cos(g11 + t5)*sin(alpha11)*sin(t11)*sin(t3) - d3*cos(t11)*cos(t4)*cos(g11 + t5)*sin(t2)*sin(t3) - r11*cos(t11)*cos(t2)*cos(t3)*sin(alpha11)*sin(t4) - d11*cos(alpha11)*cos(t2)*cos(t3)*sin(t11)*sin(t4) - r11*cos(g11 + t5)*sin(t11)*sin(t2)*sin(t4) - b11*cos(alpha11)*cos(g11 + t5)*sin(t11)*sin(t2)*sin(t4) - r5*cos(alpha11)*cos(g11 + t5)*sin(t11)*sin(t2)*sin(t4) + r3*cos(t2)*sin(alpha11)*sin(t11)*sin(t3)*sin(t4) - d3*sin(alpha11)*sin(t11)*sin(t2)*sin(t3)*sin(t4) + r3*cos(t11)*cos(t2)*cos(t3)*sin(g11 + t5) + b11*cos(t11)*cos(t2)*cos(t3)*cos(t4)*sin(g11 + t5) + r5*cos(t11)*cos(t2)*cos(t3)*cos(t4)*sin(g11 + t5) + r11*cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*cos(t4)*sin(g11 + t5) - d11*cos(t2)*cos(t3)*cos(t4)*sin(alpha11)*sin(t11)*sin(g11 + t5) - d3*cos(t11)*cos(t3)*sin(t2)*sin(g11 + t5) - r11*cos(t2)*sin(t11)*sin(t3)*sin(g11 + t5) - b11*cos(alpha11)*cos(t2)*sin(t11)*sin(t3)*sin(g11 + t5) - r5*cos(alpha11)*cos(t2)*sin(t11)*sin(t3)*sin(g11 + t5) - r3*cos(alpha11)*cos(t2)*cos(t4)*sin(t11)*sin(t3)*sin(g11 + t5) + d3*cos(alpha11)*cos(t4)*sin(t11)*sin(t2)*sin(t3)*sin(g11 + t5) - b11*cos(t11)*sin(t2)*sin(t4)*sin(g11 + t5) - r5*cos(t11)*sin(t2)*sin(t4)*sin(g11 + t5) - r11*cos(alpha11)*cos(t11)*sin(t2)*sin(t4)*sin(g11 + t5) + d11*sin(alpha11)*sin(t11)*sin(t2)*sin(t4)*sin(g11 + t5);

WA(1,2) = -(b11*cos(t11)*cos(t3)*cos(g11 + t5)) - r5*cos(t11)*cos(t3)*cos(g11 + t5) - r11*cos(alpha11)*cos(t11)*cos(t3)*cos(g11 + t5) - r3*cos(t11)*cos(t3)*cos(t4)*cos(g11 + t5) + d3*cos(t4)*sin(alpha11)*sin(t11) + d11*cos(t3)*cos(g11 + t5)*sin(alpha11)*sin(t11) + r3*cos(alpha11)*cos(g11 + t5)*sin(t11)*sin(t3) + r11*cos(t4)*cos(g11 + t5)*sin(t11)*sin(t3) + b11*cos(alpha11)*cos(t4)*cos(g11 + t5)*sin(t11)*sin(t3) + r5*cos(alpha11)*cos(t4)*cos(g11 + t5)*sin(t11)*sin(t3) - d3*cos(t11)*cos(g11 + t5)*sin(t4) - r3*cos(t3)*sin(alpha11)*sin(t11)*sin(t4) - r11*cos(t11)*sin(alpha11)*sin(t3)*sin(t4) - d11*cos(alpha11)*sin(t11)*sin(t3)*sin(t4) + r11*cos(t3)*sin(t11)*sin(g11 + t5) + b11*cos(alpha11)*cos(t3)*sin(t11)*sin(g11 + t5) + r5*cos(alpha11)*cos(t3)*sin(t11)*sin(g11 + t5) + r3*cos(alpha11)*cos(t3)*cos(t4)*sin(t11)*sin(g11 + t5) + r3*cos(t11)*sin(t3)*sin(g11 + t5) + b11*cos(t11)*cos(t4)*sin(t3)*sin(g11 + t5) + r5*cos(t11)*cos(t4)*sin(t3)*sin(g11 + t5) + r11*cos(alpha11)*cos(t11)*cos(t4)*sin(t3)*sin(g11 + t5) - d11*cos(t4)*sin(alpha11)*sin(t11)*sin(t3)*sin(g11 + t5) + d3*cos(alpha11)*sin(t11)*sin(t4)*sin(g11 + t5);

WA(1,3) = -(r11*cos(t11)*cos(t4)*sin(alpha11)) - d11*cos(alpha11)*cos(t4)*sin(t11) - r11*cos(g11 + t5)*sin(t11)*sin(t4) - b11*cos(alpha11)*cos(g11 + t5)*sin(t11)*sin(t4) - r5*cos(alpha11)*cos(g11 + t5)*sin(t11)*sin(t4) - b11*cos(t11)*sin(t4)*sin(g11 + t5) - r5*cos(t11)*sin(t4)*sin(g11 + t5) - r11*cos(alpha11)*cos(t11)*sin(t4)*sin(g11 + t5) + d11*sin(alpha11)*sin(t11)*sin(t4)*sin(g11 + t5);

WA(1,4) = -(b11*cos(t11)*cos(g11 + t5)) - r5*cos(t11)*cos(g11 + t5) - r11*cos(alpha11)*cos(t11)*cos(g11 + t5) + d11*cos(g11 + t5)*sin(alpha11)*sin(t11) + r11*sin(t11)*sin(g11 + t5) + b11*cos(alpha11)*sin(t11)*sin(g11 + t5) + r5*cos(alpha11)*sin(t11)*sin(g11 + t5);

WA(2,1) = r3*cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*cos(g11 + t5) + r11*cos(t11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5) + b11*cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5) + r5*cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5) - d11*cos(alpha11)*cos(t11)*cos(t4)*sin(t2) - d3*cos(alpha11)*cos(t11)*cos(t3)*cos(g11 + t5)*sin(t2) + r11*cos(t4)*sin(alpha11)*sin(t11)*sin(t2) - d11*cos(t11)*cos(t2)*cos(g11 + t5)*sin(alpha11)*sin(t3) - b11*cos(t2)*cos(g11 + t5)*sin(t11)*sin(t3) - r5*cos(t2)*cos(g11 + t5)*sin(t11)*sin(t3) - r11*cos(alpha11)*cos(t2)*cos(g11 + t5)*sin(t11)*sin(t3) - r3*cos(t2)*cos(t4)*cos(g11 + t5)*sin(t11)*sin(t3) + d3*cos(t4)*cos(g11 + t5)*sin(t11)*sin(t2)*sin(t3) - d11*cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*sin(t4) + r11*cos(t2)*cos(t3)*sin(alpha11)*sin(t11)*sin(t4) - r11*cos(t11)*cos(g11 + t5)*sin(t2)*sin(t4) - b11*cos(alpha11)*cos(t11)*cos(g11 + t5)*sin(t2)*sin(t4) - r5*cos(alpha11)*cos(t11)*cos(g11 + t5)*sin(t2)*sin(t4) + r3*cos(t11)*cos(t2)*sin(alpha11)*sin(t3)*sin(t4) - d3*cos(t11)*sin(alpha11)*sin(t2)*sin(t3)*sin(t4) - d11*cos(t11)*cos(t2)*cos(t3)*cos(t4)*sin(alpha11)*sin(g11 + t5) - r3*cos(t2)*cos(t3)*sin(t11)*sin(g11 + t5) - b11*cos(t2)*cos(t3)*cos(t4)*sin(t11)*sin(g11 + t5) - r5*cos(t2)*cos(t3)*cos(t4)*sin(t11)*sin(g11 + t5) - r11*cos(alpha11)*cos(t2)*cos(t3)*cos(t4)*sin(t11)*sin(g11 + t5) + d3*cos(t3)*sin(t11)*sin(t2)*sin(g11 + t5) - r11*cos(t11)*cos(t2)*sin(t3)*sin(g11 + t5) - b11*cos(alpha11)*cos(t11)*cos(t2)*sin(t3)*sin(g11 + t5) - r5*cos(alpha11)*cos(t11)*cos(t2)*sin(t3)*sin(g11 + t5) - r3*cos(alpha11)*cos(t11)*cos(t2)*cos(t4)*sin(t3)*sin(g11 + t5) + d3*cos(alpha11)*cos(t11)*cos(t4)*sin(t2)*sin(t3)*sin(g11 + t5) + d11*cos(t11)*sin(alpha11)*sin(t2)*sin(t4)*sin(g11 + t5) + b11*sin(t11)*sin(t2)*sin(t4)*sin(g11 + t5) + r5*sin(t11)*sin(t2)*sin(t4)*sin(g11 + t5) + r11*cos(alpha11)*sin(t11)*sin(t2)*sin(t4)*sin(g11 + t5);

WA(2,2) = d3*cos(t11)*cos(t4)*sin(alpha11) + d11*cos(t11)*cos(t3)*cos(g11 + t5)*sin(alpha11) + b11*cos(t3)*cos(g11 + t5)*sin(t11) + r5*cos(t3)*cos(g11 + t5)*sin(t11) + r11*cos(alpha11)*cos(t3)*cos(g11 + t5)*sin(t11) + r3*cos(t3)*cos(t4)*cos(g11 + t5)*sin(t11) + r3*cos(alpha11)*cos(t11)*cos(g11 + t5)*sin(t3) + r11*cos(t11)*cos(t4)*cos(g11 + t5)*sin(t3) + b11*cos(alpha11)*cos(t11)*cos(t4)*cos(g11 + t5)*sin(t3) + r5*cos(alpha11)*cos(t11)*cos(t4)*cos(g11 + t5)*sin(t3) - r3*cos(t11)*cos(t3)*sin(alpha11)*sin(t4) + d3*cos(g11 + t5)*sin(t11)*sin(t4) - d11*cos(alpha11)*cos(t11)*sin(t3)*sin(t4) + r11*sin(alpha11)*sin(t11)*sin(t3)*sin(t4) + r11*cos(t11)*cos(t3)*sin(g11 + t5) + b11*cos(alpha11)*cos(t11)*cos(t3)*sin(g11 + t5) + r5*cos(alpha11)*cos(t11)*cos(t3)*sin(g11 + t5) + r3*cos(alpha11)*cos(t11)*cos(t3)*cos(t4)*sin(g11 + t5) - d11*cos(t11)*cos(t4)*sin(alpha11)*sin(t3)*sin(g11 + t5) - r3*sin(t11)*sin(t3)*sin(g11 + t5) - b11*cos(t4)*sin(t11)*sin(t3)*sin(g11 + t5) - r5*cos(t4)*sin(t11)*sin(t3)*sin(g11 + t5) - r11*cos(alpha11)*cos(t4)*sin(t11)*sin(t3)*sin(g11 + t5) + d3*cos(alpha11)*cos(t11)*sin(t4)*sin(g11 + t5);

WA(2,3) = -(d11*cos(alpha11)*cos(t11)*cos(t4)) + r11*cos(t4)*sin(alpha11)*sin(t11) - r11*cos(t11)*cos(g11 + t5)*sin(t4) - b11*cos(alpha11)*cos(t11)*cos(g11 + t5)*sin(t4) - r5*cos(alpha11)*cos(t11)*cos(g11 + t5)*sin(t4) + d11*cos(t11)*sin(alpha11)*sin(t4)*sin(g11 + t5) + b11*sin(t11)*sin(t4)*sin(g11 + t5) + r5*sin(t11)*sin(t4)*sin(g11 + t5) + r11*cos(alpha11)*sin(t11)*sin(t4)*sin(g11 + t5);

WA(2,4) = d11*cos(t11)*cos(g11 + t5)*sin(alpha11) + b11*cos(g11 + t5)*sin(t11) + r5*cos(g11 + t5)*sin(t11) + r11*cos(alpha11)*cos(g11 + t5)*sin(t11) + r11*cos(t11)*sin(g11 + t5) + b11*cos(alpha11)*cos(t11)*sin(g11 + t5) + r5*cos(alpha11)*cos(t11)*sin(g11 + t5);

WA(3,1) = -(r3*cos(t2)*cos(t3)*cos(g11 + t5)*sin(alpha11)) - b11*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(alpha11) - r5*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(alpha11) + d11*cos(t4)*sin(alpha11)*sin(t2) + d3*cos(t3)*cos(g11 + t5)*sin(alpha11)*sin(t2) - d11*cos(alpha11)*cos(t2)*cos(g11 + t5)*sin(t3) + d11*cos(t2)*cos(t3)*sin(alpha11)*sin(t4) + b11*cos(g11 + t5)*sin(alpha11)*sin(t2)*sin(t4) + r5*cos(g11 + t5)*sin(alpha11)*sin(t2)*sin(t4) + r3*cos(alpha11)*cos(t2)*sin(t3)*sin(t4) - d3*cos(alpha11)*sin(t2)*sin(t3)*sin(t4) - d11*cos(alpha11)*cos(t2)*cos(t3)*cos(t4)*sin(g11 + t5) + b11*cos(t2)*sin(alpha11)*sin(t3)*sin(g11 + t5) + r5*cos(t2)*sin(alpha11)*sin(t3)*sin(g11 + t5) + r3*cos(t2)*cos(t4)*sin(alpha11)*sin(t3)*sin(g11 + t5) - d3*cos(t4)*sin(alpha11)*sin(t2)*sin(t3)*sin(g11 + t5) + d11*cos(alpha11)*sin(t2)*sin(t4)*sin(g11 + t5);

WA(3,2) = d3*cos(alpha11)*cos(t4) + d11*cos(alpha11)*cos(t3)*cos(g11 + t5) - r3*cos(g11 + t5)*sin(alpha11)*sin(t3) - b11*cos(t4)*cos(g11 + t5)*sin(alpha11)*sin(t3) - r5*cos(t4)*cos(g11 + t5)*sin(alpha11)*sin(t3) - r3*cos(alpha11)*cos(t3)*sin(t4) + d11*sin(alpha11)*sin(t3)*sin(t4) - b11*cos(t3)*sin(alpha11)*sin(g11 + t5) - r5*cos(t3)*sin(alpha11)*sin(g11 + t5) - r3*cos(t3)*cos(t4)*sin(alpha11)*sin(g11 + t5) - d11*cos(alpha11)*cos(t4)*sin(t3)*sin(g11 + t5) - d3*sin(alpha11)*sin(t4)*sin(g11 + t5);

WA(3,3) = d11*cos(t4)*sin(alpha11) + b11*cos(g11 + t5)*sin(alpha11)*sin(t4) + r5*cos(g11 + t5)*sin(alpha11)*sin(t4) + d11*cos(alpha11)*sin(t4)*sin(g11 + t5);

WA(3,4) = d11*cos(alpha11)*cos(g11 + t5) - b11*sin(alpha11)*sin(g11 + t5) - r5*sin(alpha11)*sin(g11 + t5);

WA(4,1) = -(cos(t11)*cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)) - cos(t4)*sin(alpha11)*sin(t11)*sin(t2) + cos(alpha11)*cos(t2)*cos(g11 + t5)*sin(t11)*sin(t3) - cos(t2)*cos(t3)*sin(alpha11)*sin(t11)*sin(t4) + cos(t11)*cos(g11 + t5)*sin(t2)*sin(t4) + cos(alpha11)*cos(t2)*cos(t3)*cos(t4)*sin(t11)*sin(g11 + t5) + cos(t11)*cos(t2)*sin(t3)*sin(g11 + t5) - cos(alpha11)*sin(t11)*sin(t2)*sin(t4)*sin(g11 + t5);

WA(4,2) = -(cos(alpha11)*cos(t3)*cos(g11 + t5)*sin(t11)) - cos(t11)*cos(t4)*cos(g11 + t5)*sin(t3) - sin(alpha11)*sin(t11)*sin(t3)*sin(t4) - cos(t11)*cos(t3)*sin(g11 + t5) + cos(alpha11)*cos(t4)*sin(t11)*sin(t3)*sin(g11 + t5);

WA(4,3) = -(cos(t4)*sin(alpha11)*sin(t11)) + cos(t11)*cos(g11 + t5)*sin(t4) - cos(alpha11)*sin(t11)*sin(t4)*sin(g11 + t5);

WA(4,4) = -(cos(alpha11)*cos(g11 + t5)*sin(t11)) - cos(t11)*sin(g11 + t5);

WA(5,1) = cos(t2)*cos(t3)*cos(t4)*cos(g11 + t5)*sin(t11) - cos(t11)*cos(t4)*sin(alpha11)*sin(t2) + cos(alpha11)*cos(t11)*cos(t2)*cos(g11 + t5)*sin(t3) - cos(t11)*cos(t2)*cos(t3)*sin(alpha11)*sin(t4) - cos(g11 + t5)*sin(t11)*sin(t2)*sin(t4) + cos(alpha11)*cos(t11)*cos(t2)*cos(t3)*cos(t4)*sin(g11 + t5) - cos(t2)*sin(t11)*sin(t3)*sin(g11 + t5) - cos(alpha11)*cos(t11)*sin(t2)*sin(t4)*sin(g11 + t5);

WA(5,2) = -(cos(alpha11)*cos(t11)*cos(t3)*cos(g11 + t5)) + cos(t4)*cos(g11 + t5)*sin(t11)*sin(t3) - cos(t11)*sin(alpha11)*sin(t3)*sin(t4) + cos(t3)*sin(t11)*sin(g11 + t5) + cos(alpha11)*cos(t11)*cos(t4)*sin(t3)*sin(g11 + t5);

WA(5,3) = -(cos(t11)*cos(t4)*sin(alpha11)) - cos(g11 + t5)*sin(t11)*sin(t4) - cos(alpha11)*cos(t11)*sin(t4)*sin(g11 + t5);

WA(5,4) = -(cos(alpha11)*cos(t11)*cos(g11 + t5)) + sin(t11)*sin(g11 + t5);

   WP=zeros( 5 , 5 );

WP(1,1) = -(r11*cos(t11)*sin(alpha11)) - d11*cos(alpha11)*sin(t11);

WP(1,2) = -(r8*cos(t7)*cos(t8)*sin(t10)) - r10*cos(t7)*cos(t8)*cos(t9)*sin(t10) + d8*cos(t8)*sin(t10)*sin(t7) - r10*cos(t10)*cos(t7)*sin(t8) - r8*cos(t10)*cos(t7)*cos(t9)*sin(t8) + d8*cos(t10)*cos(t9)*sin(t7)*sin(t8) + r10*sin(t10)*sin(t7)*sin(t9);

WP(1,3) = r10*cos(t10)*cos(t8) + r8*cos(t10)*cos(t8)*cos(t9) - r8*sin(t10)*sin(t8) - r10*cos(t9)*sin(t10)*sin(t8) + d8*cos(t10)*sin(t9);

WP(1,4) = r10*sin(t10)*sin(t9);

WP(1,5) = r10*cos(t10);

WP(2,1) = -(d11*cos(alpha11)*cos(t11)) + r11*sin(alpha11)*sin(t11);

WP(2,2) = -(r8*cos(t10)*cos(t7)*cos(t8)) - r10*cos(t10)*cos(t7)*cos(t8)*cos(t9) + d8*cos(t10)*cos(t8)*sin(t7) + r10*cos(t7)*sin(t10)*sin(t8) + r8*cos(t7)*cos(t9)*sin(t10)*sin(t8) - d8*cos(t9)*sin(t10)*sin(t7)*sin(t8) + r10*cos(t10)*sin(t7)*sin(t9);

WP(2,3) = -(r10*cos(t8)*sin(t10)) - r8*cos(t8)*cos(t9)*sin(t10) - r8*cos(t10)*sin(t8) - r10*cos(t10)*cos(t9)*sin(t8) - d8*sin(t10)*sin(t9);

WP(2,4) = r10*cos(t10)*sin(t9);

WP(2,5) = -(r10*sin(t10));

WP(3,1) = d11*sin(alpha11);

WP(3,2) = -(r8*cos(t7)*sin(t8)*sin(t9)) + d8*sin(t7)*sin(t8)*sin(t9);

WP(3,3) = -(d8*cos(t9)) + r8*cos(t8)*sin(t9);

WP(3,4) = 0;

WP(3,5) = 0;

WP(4,1) = -(sin(alpha11)*sin(t11));

WP(4,2) = cos(t10)*cos(t7)*cos(t8)*cos(t9) - cos(t7)*sin(t10)*sin(t8) - cos(t10)*sin(t7)*sin(t9);

WP(4,3) = cos(t8)*sin(t10) + cos(t10)*cos(t9)*sin(t8);

WP(4,4) = -(cos(t10)*sin(t9));

WP(4,5) = sin(t10);

WP(5,1) = -(cos(t11)*sin(alpha11));

WP(5,2) = -(cos(t7)*cos(t8)*cos(t9)*sin(t10)) - cos(t10)*cos(t7)*sin(t8) + sin(t10)*sin(t7)*sin(t9);

WP(5,3) = cos(t10)*cos(t8) - cos(t9)*sin(t10)*sin(t8);

WP(5,4) = sin(t10)*sin(t9);

WP(5,5) = cos(t10);

    WAC=zeros( 1 , 4 );

WAC(1,1) = -(cos(alpha11)*cos(t4)*sin(t2)) - cos(t2)*cos(g11 + t5)*sin(alpha11)*sin(t3) - cos(alpha11)*cos(t2)*cos(t3)*sin(t4) - cos(t2)*cos(t3)*cos(t4)*sin(alpha11)*sin(g11 + t5) + sin(alpha11)*sin(t2)*sin(t4)*sin(g11 + t5);

WAC(1,2) = cos(t3)*cos(g11 + t5)*sin(alpha11) - cos(alpha11)*sin(t3)*sin(t4) - cos(t4)*sin(alpha11)*sin(t3)*sin(g11 + t5);

WAC(1,3) = -(cos(alpha11)*cos(t4)) + sin(alpha11)*sin(t4)*sin(g11 + t5);

WAC(1,4) = cos(g11 + t5)*sin(alpha11);
 WPC=zeros( 1 , 5 );

WPC(1,1) = -cos(alpha11);

WPC(1,2) = cos(t9)*sin(t7) + cos(t7)*cos(t8)*sin(t9);

WPC(1,3) = sin(t8)*sin(t9);

WPC(1,4) = cos(t9);

WPC(1,5) = 0;


WC=zeros( 1 , 1 );

WC(1,1) = 1;

J1=[WA WP zeros(5,1)];
J2=[WAC WPC WC];
J=[J1;J2];
