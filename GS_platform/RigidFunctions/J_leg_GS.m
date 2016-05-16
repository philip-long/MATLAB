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
%   Name of file : D:/Documents/functions/SYMORO/GS/GS_leg1.jac
%
%
%
%
%     Geometric parameters   
%
%
%j     ant   mu    sigma gamma b     alpha d     theta r
%
%                                    -pi
%1     0     1     0     g1    f1    ---   d1    t1    0
%                                     2
%                                    pi
%2     1     1     0     0     0     --    0     t2    0
%                                    2
%                                    pi
%3     2     1     1     0     0     --    0     0     r3
%                                    2
%
%Size=( 6 , 3 );
%Jacobian Matrix of frame 3 projected  into frame 0. Intermediate frame = 3
function J=J_leg_GS(q,l)

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



J(1,1) = -(r3*cos(g1)*sin(t1)*sin(t2));

J(2,1) = -(r3*sin(g1)*sin(t1)*sin(t2));

J(3,1) = -(r3*cos(t1)*sin(t2));

J(4,1) = -sin(g1);

J(5,1) = cos(g1);

J(6,1) =0;

J(1,2) = r3*cos(g1)*cos(t1)*cos(t2) - r3*sin(g1)*sin(t2);

J(2,2) = r3*cos(t1)*cos(t2)*sin(g1) + r3*cos(g1)*sin(t2);

J(3,2) = -(r3*cos(t2)*sin(t1));

J(4,2) = cos(g1)*sin(t1);

J(5,2) = sin(g1)*sin(t1);

J(6,2) = cos(t1);

J(1,3) = cos(t2)*sin(g1) + cos(g1)*cos(t1)*sin(t2);

J(2,3) = -(cos(g1)*cos(t2)) + cos(t1)*sin(g1)*sin(t2);

J(3,3) = -(sin(t1)*sin(t2));

J(4,3) =0;

J(5,3) =0;

J(6,3) =0;


J=J(1:3,1:3);

