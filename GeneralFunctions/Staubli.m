%%****************************************** ;
%%* SYMORO+ : SYmbolic MOdelling of RObots * ;
%%*========================================* ;
%%*      IRCCyN-ECN - 1, rue de la Noe     * ;
%%*      B.P.92101                         * ;
%%*      44321 Nantes cedex 3, FRANCE      * ;
%%*      www.irccyn.ec-nantes.fr           * ;
%%****************************************** ;
%
%
%   Name of file : D:/Mes documents/functions/SYMORO tests/Staubli.jac
%
%
%
%
%     Geometric parameters   
%
%
%j     ant   mu    sigma gamma b     alpha d     theta r
%
%
%1     0     0     0     0     0     0     0     t1    0
%
%                                    pi
%2     1     0     0     0     0     --    0     t2    0
%                                    2
%
%3     2     0     0     0     0     0     d3    t3    0
%
%                                    -pi
%4     3     0     0     0     0     ---   0     t4    RL4
%                                     2
%                                    pi
%5     4     0     0     0     0     --    0     t5    0
%                                    2
%                                    -pi
%6     5     0     0     0     0     ---   0     t6    0
%                                     2
%
%Size=( 6 , 6 );
%Jacobian Matrix of frame 6 projected  into frame 0. Intermediate frame = 6
function J=Staubli(u);

t1=u(1);
t2=u(2);
t3=u(3);
t4=u(4);
t5=u(5);
t6=u(6);


J(1,1) = -(d3*cos(t2)*sin(t1)) + RL4*sin(t1)*sin(t2 + t3);

J(2,1) = d3*cos(t1)*cos(t2) - RL4*cos(t1)*sin(t2 + t3);

J(3,1) =0;

J(4,1) =0;

J(5,1) =0;

J(6,1) =1;

J(1,2) = -(RL4*cos(t1)*cos(t2 + t3)) - d3*cos(t1)*sin(t2);

J(2,2) = -(RL4*cos(t2 + t3)*sin(t1)) - d3*sin(t1)*sin(t2);

J(3,2) = d3*cos(t2) - RL4*sin(t2 + t3);

J(4,2) = sin(t1);

J(5,2) = -cos(t1);

J(6,2) =0;

J(1,3) = -(RL4*cos(t1)*cos(t2 + t3));

J(2,3) = -(RL4*cos(t2 + t3)*sin(t1));

J(3,3) = -(RL4*sin(t2 + t3));

J(4,3) = sin(t1);

J(5,3) = -cos(t1);

J(6,3) =0;

J(1,4) =0;

J(2,4) =0;

J(3,4) =0;

J(4,4) = -(cos(t1)*sin(t2 + t3));

J(5,4) = -(sin(t1)*sin(t2 + t3));

J(6,4) = cos(t2 + t3);

J(1,5) =0;

J(2,5) =0;

J(3,5) =0;

J(4,5) = cos(t4)*sin(t1) + cos(t1)*cos(t2 + t3)*sin(t4);

J(5,5) = -(cos(t1)*cos(t4)) + cos(t2 + t3)*sin(t1)*sin(t4);

J(6,5) = sin(t2 + t3)*sin(t4);

J(1,6) =0;

J(2,6) =0;

J(3,6) =0;

J(4,6) = -(cos(t1)*cos(t5)*sin(t2 + t3))-...
  cos(t1)*cos(t2 + t3)*cos(t4)*sin(t5) + sin(t1)*sin(t4)*sin(t5);

J(5,6) = -(cos(t5)*sin(t1)*sin(t2 + t3))-...
  cos(t2 + t3)*cos(t4)*sin(t1)*sin(t5) - cos(t1)*sin(t4)*sin(t5);

J(6,6) = cos(t2 + t3)*cos(t5) - cos(t4)*sin(t2 + t3)*sin(t5);




