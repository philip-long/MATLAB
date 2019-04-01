function Q = qMultiply( Q1, Q2 )
% qMultiply: A modified version of qmult2 by Przemyslaw Baranski
%            modification copes with symbolic variables.
% IN: 
%     Q1 - first quaternion
%     Q2 - second quaternion
% 
% OUT:
%     Q - output quaternion, Q = Q1*Q2
%     
% REMARKS:
%     1) Quaternion multiplication is not commutative, i.e. Q1*Q2 != Q2*Q1
%     2) Quaternion multiplication is associative, i.e. Q1*Q2*Q3 = Q1*(Q2*Q3)=(Q1*Q2)*Q3
% 
% VERSION: 03.03.2012
s1 = Q1(1);
s2 = Q2(1);
v1 = Q1(2:4);
v2 = Q2(2:4);
s =s1*s2 - transpose(v1)*v2;
v = s1*v2 + s2*v1 + skew(v1)*v2;
v = reshape( v, 3, 1 );
Q = [s;v];
end