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
%   Name of file : D:/Mes documents/KUKA/SymoroFiles/KukalwrAmine.jac
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
%1     0     1     0     0     0     0     0     t1    r1
%
%                                    pi
%2     1     1     0     0     0     --    0     t2    0
%                                    2
%                                    -pi
%3     2     1     0     0     0     ---   0     t3    r3
%                                     2
%                                    -pi
%4     3     1     0     0     0     ---   0     t4    0
%                                     2
%                                    pi
%5     4     1     0     0     0     --    0     t5    r5
%                                    2
%                                    pi
%6     5     1     0     0     0     --    0     t6    0
%                                    2
%                                    -pi
%7     6     1     0     0     0     ---   0     t7    r7
%                                     2
%
%Size=( 6 , 7 );
%Jacobian Matrix of frame 7 projected  into frame 0. Intermediate frame = 7
function J=J70(u)

r1=0;
r3=0.4;
r5=0.39;
r7=0;
t1=u(1);
t2=u(2);
t3=u(3);
t4=u(4);
t5=u(5);
t6=u(6);
t7=u(7);


J(1,1) = r3*sin(t1)*sin(t2) + r5*cos(t4)*sin(t1)*sin(t2)+...
  r7*cos(t4)*cos(t6)*sin(t1)*sin(t2)-...
  r5*cos(t2)*cos(t3)*sin(t1)*sin(t4)-...
  r7*cos(t2)*cos(t3)*cos(t6)*sin(t1)*sin(t4)-...
  r5*cos(t1)*sin(t3)*sin(t4) - r7*cos(t1)*cos(t6)*sin(t3)*sin(t4)+...
  r7*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1)*sin(t6)+...
  r7*cos(t1)*cos(t4)*cos(t5)*sin(t3)*sin(t6)+...
  r7*cos(t5)*sin(t1)*sin(t2)*sin(t4)*sin(t6)+...
  r7*cos(t1)*cos(t3)*sin(t5)*sin(t6)-...
  r7*cos(t2)*sin(t1)*sin(t3)*sin(t5)*sin(t6);

J(2,1) = -(r3*cos(t1)*sin(t2)) - r5*cos(t1)*cos(t4)*sin(t2)-...
  r7*cos(t1)*cos(t4)*cos(t6)*sin(t2)+...
  r5*cos(t1)*cos(t2)*cos(t3)*sin(t4)+...
  r7*cos(t1)*cos(t2)*cos(t3)*cos(t6)*sin(t4)-...
  r5*sin(t1)*sin(t3)*sin(t4) - r7*cos(t6)*sin(t1)*sin(t3)*sin(t4)-...
  r7*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t6)+...
  r7*cos(t4)*cos(t5)*sin(t1)*sin(t3)*sin(t6)-...
  r7*cos(t1)*cos(t5)*sin(t2)*sin(t4)*sin(t6)+...
  r7*cos(t3)*sin(t1)*sin(t5)*sin(t6)+...
  r7*cos(t1)*cos(t2)*sin(t3)*sin(t5)*sin(t6);

J(3,1) =0;

J(4,1) =0;

J(5,1) =0;

J(6,1) =1;

J(1,2) = -(r3*cos(t1)*cos(t2)) - r5*cos(t1)*cos(t2)*cos(t4)-...
  r7*cos(t1)*cos(t2)*cos(t4)*cos(t6)-...
  r5*cos(t1)*cos(t3)*sin(t2)*sin(t4)-...
  r7*cos(t1)*cos(t3)*cos(t6)*sin(t2)*sin(t4)+...
  r7*cos(t1)*cos(t3)*cos(t4)*cos(t5)*sin(t2)*sin(t6)-...
  r7*cos(t1)*cos(t2)*cos(t5)*sin(t4)*sin(t6)-...
  r7*cos(t1)*sin(t2)*sin(t3)*sin(t5)*sin(t6);

J(2,2) = -(r3*cos(t2)*sin(t1)) - r5*cos(t2)*cos(t4)*sin(t1)-...
  r7*cos(t2)*cos(t4)*cos(t6)*sin(t1)-...
  r5*cos(t3)*sin(t1)*sin(t2)*sin(t4)-...
  r7*cos(t3)*cos(t6)*sin(t1)*sin(t2)*sin(t4)+...
  r7*cos(t3)*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t6)-...
  r7*cos(t2)*cos(t5)*sin(t1)*sin(t4)*sin(t6)-...
  r7*sin(t1)*sin(t2)*sin(t3)*sin(t5)*sin(t6);

J(3,2) = -(r3*sin(t2)) - r5*cos(t4)*sin(t2) - r7*cos(t4)*cos(t6)*sin(t2)+...
  r5*cos(t2)*cos(t3)*sin(t4) + r7*cos(t2)*cos(t3)*cos(t6)*sin(t4)-...
  r7*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t6)-...
  r7*cos(t5)*sin(t2)*sin(t4)*sin(t6)+...
  r7*cos(t2)*sin(t3)*sin(t5)*sin(t6);

J(4,2) = sin(t1);

J(5,2) = -cos(t1);

J(6,2) =0;

J(1,3) = -(r5*cos(t3)*sin(t1)*sin(t4)) - r7*cos(t3)*cos(t6)*sin(t1)*sin(t4)-...
  r5*cos(t1)*cos(t2)*sin(t3)*sin(t4)-...
  r7*cos(t1)*cos(t2)*cos(t6)*sin(t3)*sin(t4)+...
  r7*cos(t3)*cos(t4)*cos(t5)*sin(t1)*sin(t6)+...
  r7*cos(t1)*cos(t2)*cos(t4)*cos(t5)*sin(t3)*sin(t6)+...
  r7*cos(t1)*cos(t2)*cos(t3)*sin(t5)*sin(t6)-...
  r7*sin(t1)*sin(t3)*sin(t5)*sin(t6);

J(2,3) = r5*cos(t1)*cos(t3)*sin(t4) + r7*cos(t1)*cos(t3)*cos(t6)*sin(t4)-...
  r5*cos(t2)*sin(t1)*sin(t3)*sin(t4)-...
  r7*cos(t2)*cos(t6)*sin(t1)*sin(t3)*sin(t4)-...
  r7*cos(t1)*cos(t3)*cos(t4)*cos(t5)*sin(t6)+...
  r7*cos(t2)*cos(t4)*cos(t5)*sin(t1)*sin(t3)*sin(t6)+...
  r7*cos(t2)*cos(t3)*sin(t1)*sin(t5)*sin(t6)+...
  r7*cos(t1)*sin(t3)*sin(t5)*sin(t6);

J(3,3) = -(r5*sin(t2)*sin(t3)*sin(t4)) - r7*cos(t6)*sin(t2)*sin(t3)*sin(t4)+...
  r7*cos(t4)*cos(t5)*sin(t2)*sin(t3)*sin(t6)+...
  r7*cos(t3)*sin(t2)*sin(t5)*sin(t6);

J(4,3) = -(cos(t1)*sin(t2));

J(5,3) = -(sin(t1)*sin(t2));

J(6,3) = cos(t2);

J(1,4) = r5*cos(t1)*cos(t2)*cos(t3)*cos(t4)+...
  r7*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t6)-...
  r5*cos(t4)*sin(t1)*sin(t3) - r7*cos(t4)*cos(t6)*sin(t1)*sin(t3)+...
  r5*cos(t1)*sin(t2)*sin(t4) + r7*cos(t1)*cos(t6)*sin(t2)*sin(t4)-...
  r7*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t6)+...
  r7*cos(t1)*cos(t2)*cos(t3)*cos(t5)*sin(t4)*sin(t6)-...
  r7*cos(t5)*sin(t1)*sin(t3)*sin(t4)*sin(t6);

J(2,4) = r5*cos(t2)*cos(t3)*cos(t4)*sin(t1)+...
  r7*cos(t2)*cos(t3)*cos(t4)*cos(t6)*sin(t1)+...
  r5*cos(t1)*cos(t4)*sin(t3) + r7*cos(t1)*cos(t4)*cos(t6)*sin(t3)+...
  r5*sin(t1)*sin(t2)*sin(t4) + r7*cos(t6)*sin(t1)*sin(t2)*sin(t4)-...
  r7*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t6)+...
  r7*cos(t2)*cos(t3)*cos(t5)*sin(t1)*sin(t4)*sin(t6)+...
  r7*cos(t1)*cos(t5)*sin(t3)*sin(t4)*sin(t6);

J(3,4) = r5*cos(t3)*cos(t4)*sin(t2) + r7*cos(t3)*cos(t4)*cos(t6)*sin(t2)-...
  r5*cos(t2)*sin(t4) - r7*cos(t2)*cos(t6)*sin(t4)+...
  r7*cos(t2)*cos(t4)*cos(t5)*sin(t6)+...
  r7*cos(t3)*cos(t5)*sin(t2)*sin(t4)*sin(t6);

J(4,4) = -(cos(t3)*sin(t1)) - cos(t1)*cos(t2)*sin(t3);

J(5,4) = cos(t1)*cos(t3) - cos(t2)*sin(t1)*sin(t3);

J(6,4) = -(sin(t2)*sin(t3));

J(1,5) = r7*cos(t3)*cos(t5)*sin(t1)*sin(t6)+...
  r7*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t6)+...
  r7*cos(t1)*cos(t2)*cos(t3)*cos(t4)*sin(t5)*sin(t6)-...
  r7*cos(t4)*sin(t1)*sin(t3)*sin(t5)*sin(t6)+...
  r7*cos(t1)*sin(t2)*sin(t4)*sin(t5)*sin(t6);

J(2,5) = -(r7*cos(t1)*cos(t3)*cos(t5)*sin(t6))+...
  r7*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t6)+...
  r7*cos(t2)*cos(t3)*cos(t4)*sin(t1)*sin(t5)*sin(t6)+...
  r7*cos(t1)*cos(t4)*sin(t3)*sin(t5)*sin(t6)+...
  r7*sin(t1)*sin(t2)*sin(t4)*sin(t5)*sin(t6);

J(3,5) = r7*cos(t5)*sin(t2)*sin(t3)*sin(t6)+...
  r7*cos(t3)*cos(t4)*sin(t2)*sin(t5)*sin(t6)-...
  r7*cos(t2)*sin(t4)*sin(t5)*sin(t6);

J(4,5) = -(cos(t1)*cos(t4)*sin(t2)) + cos(t1)*cos(t2)*cos(t3)*sin(t4)-...
  sin(t1)*sin(t3)*sin(t4);

J(5,5) = -(cos(t4)*sin(t1)*sin(t2)) + cos(t2)*cos(t3)*sin(t1)*sin(t4)+...
  cos(t1)*sin(t3)*sin(t4);

J(6,5) = cos(t2)*cos(t4) + cos(t3)*sin(t2)*sin(t4);

J(1,6) = -(r7*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5)*cos(t6))+...
  r7*cos(t4)*cos(t5)*cos(t6)*sin(t1)*sin(t3)-...
  r7*cos(t1)*cos(t5)*cos(t6)*sin(t2)*sin(t4)+...
  r7*cos(t3)*cos(t6)*sin(t1)*sin(t5)+...
  r7*cos(t1)*cos(t2)*cos(t6)*sin(t3)*sin(t5)+...
  r7*cos(t1)*cos(t4)*sin(t2)*sin(t6)-...
  r7*cos(t1)*cos(t2)*cos(t3)*sin(t4)*sin(t6)+...
  r7*sin(t1)*sin(t3)*sin(t4)*sin(t6);

J(2,6) = -(r7*cos(t2)*cos(t3)*cos(t4)*cos(t5)*cos(t6)*sin(t1))-...
  r7*cos(t1)*cos(t4)*cos(t5)*cos(t6)*sin(t3)-...
  r7*cos(t5)*cos(t6)*sin(t1)*sin(t2)*sin(t4)-...
  r7*cos(t1)*cos(t3)*cos(t6)*sin(t5)+...
  r7*cos(t2)*cos(t6)*sin(t1)*sin(t3)*sin(t5)+...
  r7*cos(t4)*sin(t1)*sin(t2)*sin(t6)-...
  r7*cos(t2)*cos(t3)*sin(t1)*sin(t4)*sin(t6)-...
  r7*cos(t1)*sin(t3)*sin(t4)*sin(t6);

J(3,6) = -(r7*cos(t3)*cos(t4)*cos(t5)*cos(t6)*sin(t2))+...
  r7*cos(t2)*cos(t5)*cos(t6)*sin(t4)+...
  r7*cos(t6)*sin(t2)*sin(t3)*sin(t5) - r7*cos(t2)*cos(t4)*sin(t6)-...
  r7*cos(t3)*sin(t2)*sin(t4)*sin(t6);

J(4,6) = cos(t3)*cos(t5)*sin(t1) + cos(t1)*cos(t2)*cos(t5)*sin(t3)+...
  cos(t1)*cos(t2)*cos(t3)*cos(t4)*sin(t5)-...
  cos(t4)*sin(t1)*sin(t3)*sin(t5) + cos(t1)*sin(t2)*sin(t4)*sin(t5);

J(5,6) = -(cos(t1)*cos(t3)*cos(t5)) + cos(t2)*cos(t5)*sin(t1)*sin(t3)+...
  cos(t2)*cos(t3)*cos(t4)*sin(t1)*sin(t5)+...
  cos(t1)*cos(t4)*sin(t3)*sin(t5) + sin(t1)*sin(t2)*sin(t4)*sin(t5);

J(6,6) = cos(t5)*sin(t2)*sin(t3) + cos(t3)*cos(t4)*sin(t2)*sin(t5)-...
  cos(t2)*sin(t4)*sin(t5);

J(1,7) =0;

J(2,7) =0;

J(3,7) =0;

J(4,7) = -(cos(t1)*cos(t4)*cos(t6)*sin(t2))+...
  cos(t1)*cos(t2)*cos(t3)*cos(t6)*sin(t4)-...
  cos(t6)*sin(t1)*sin(t3)*sin(t4)-...
  cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t6)+...
  cos(t4)*cos(t5)*sin(t1)*sin(t3)*sin(t6)-...
  cos(t1)*cos(t5)*sin(t2)*sin(t4)*sin(t6)+...
  cos(t3)*sin(t1)*sin(t5)*sin(t6)+...
  cos(t1)*cos(t2)*sin(t3)*sin(t5)*sin(t6);

J(5,7) = -(cos(t4)*cos(t6)*sin(t1)*sin(t2))+...
  cos(t2)*cos(t3)*cos(t6)*sin(t1)*sin(t4)+...
  cos(t1)*cos(t6)*sin(t3)*sin(t4)-...
  cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1)*sin(t6)-...
  cos(t1)*cos(t4)*cos(t5)*sin(t3)*sin(t6)-...
  cos(t5)*sin(t1)*sin(t2)*sin(t4)*sin(t6)-...
  cos(t1)*cos(t3)*sin(t5)*sin(t6)+...
  cos(t2)*sin(t1)*sin(t3)*sin(t5)*sin(t6);

J(6,7) = cos(t2)*cos(t4)*cos(t6) + cos(t3)*cos(t6)*sin(t2)*sin(t4)-...
  cos(t3)*cos(t4)*cos(t5)*sin(t2)*sin(t6)+...
  cos(t2)*cos(t5)*sin(t4)*sin(t6) + sin(t2)*sin(t3)*sin(t5)*sin(t6);





% T= T70(u);% Transformation matrix CHANGE RIGHT NOW OUTPUTTING T0E NOT T70
% P=TnE(1:3,4); %Distance to tool frame
% L=T(1:3,1:3)*P; %Change the reference frame to the world one  
% Lhat=skew(L); %Skew the matrix
% 
% J0e=[eye(3) -Lhat; zeros(3) eye(3) ]*J % Change the point of the Jacobian
% %