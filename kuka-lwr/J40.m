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
%   Name of file : D:/Documents/KUKA/SymoroFiles/KukaRedjac.jac
%
%
%
%
%     Geometric parameters   
%
%
%j        ant      mu       sigma    gamma    b        alpha    d        theta    r
%
%
%1        0        1        0        0        0        0        0        t1       r1
%
%                                                      pi                -pi
%2        1        1        0        0        0        --       0        --- + t2 0
%                                                      2                  2
%                                                      -pi
%3        2        1        0        0        0        ---      0        t3       r3
%                                                       2
%                                                      -pi
%4        3        1        0        0        0        ---      0        t4       0
%                                                       2
%
%Size=( 6 , 4 );
%Jacobian Matrix of frame 4 projected  into frame 0. Intermediate frame = 4
function J=J40(u);

global r1 r3 r5 r7 Joint2Offset
% Declaration of global output variables


% Function description:
t1=u(1);
t2=u(2)+Joint2Offset;
t3=u(3);
t4=u(4);

J(1,1) = -(r3*cos(t2)*sin(t1));

J(2,1) = r3*cos(t1)*cos(t2);

J(3,1) =0;

J(4,1) =0;

J(5,1) =0;

J(6,1) =1;

J(1,2) = -(r3*cos(t1)*sin(t2));

J(2,2) = -(r3*sin(t1)*sin(t2));

J(3,2) = r3*cos(t2);

J(4,2) = sin(t1);

J(5,2) = -cos(t1);

J(6,2) =0;

J(1,3) =0;

J(2,3) =0;

J(3,3) =0;

J(4,3) = cos(t1)*cos(t2);

J(5,3) = cos(t2)*sin(t1);

J(6,3) = sin(t2);

J(1,4) =0;

J(2,4) =0;

J(3,4) =0;

J(4,4) = -(cos(t3)*sin(t1)) - cos(t1)*sin(t2)*sin(t3);

J(5,4) = cos(t1)*cos(t3) - sin(t1)*sin(t2)*sin(t3);

J(6,4) = cos(t2)*sin(t3);







end

