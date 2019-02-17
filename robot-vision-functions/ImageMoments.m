% Image Moments interaction matrix Chaumette 2004

syms m00 m01 m10 m11 m21 m12 m22






% mvx=-ki*(A*mkikj + B*mki-1kj+1 + C*mki-1kj)-( A*mkikj)
% mvy=-kj*(A*mki+1kj-1 + B*mkikj + C*mkikj-1)-( B*mkikj)
% mvz=(ki+kj+3)*(A*mki+1kj+ B*mkikj+1+C*mkikj)-C*mkikj;
% mwx=(ki+kj+3)*mkikj+1 + kj*mkikj-1
% mwy=-(ki+kj+3)*mki+1kj - ki*mki-1kj
% mwz=ki*mki-1kj+1 - kj*mki+1kj-1


Lm=[mvx mvy mvz mwx mwy mwz]

% INTERACTION MATRIX DERIVATIONS
% a=m00, ==> ki=0 kj=0, xg*a=m10   yg*a=m01
% 1st step
mvx=-A*m00
mvy=-0*(A*m(0+1,0-1) + B*m(0,0) + C*m(0,0-1))-( B*m(0,0))
mvz=(0+0+3)*(A*m(0+1,0)+ B*m(0,0+1)+C*m(0,0))-C*m(0,0);
mwx=(0+0+3)*m(0,0+1) + 0*m(0,0-1)
mwy=-(0+0+3)*m(0+1,0) - 0*m(0-1,0-1)
mwz=0*m(0-1,0+1) - 0*m(0+1,0-1)

% substituting in and using equation of plane we
% find the ineraction matrix associated with the
% area
% subs
mvx=-( A*a)
mvy=-( B*a)
mvz=3*(A*xg*a+ B*yg*a+C*a)-C*a=a(A*xg+B*yg+C - C/3)=a(Zg
mwx=3*yg*a 
mwy=-3*xg*a
mwz=0

% lets try m10 ki=1,kj=0

mvx=-(A*m10 + B*m01 + C*m00)-( A*m10)
mvy=-(B*m10)
mvz=4*(A*m1+10+ B*m10+1+C*m10)-C*m10;
mwx=4*m11
mwy=-4*m20 - 1*m00
mwz=m01




%%

% in this case i-1 = ji i = ki i+1 = li
%              j-1 = jj j = kj j+1 = lj
%

% This is new figure
mvx=-ki*(A*mkikj + B*mjilj + C*mjikj)-( A*mkikj)
mvy=-kj*(A*mlijj + B*mkikj + C*mkijj)-( B*mkikj)
mvz=(ki+kj+3)*(A*mlikj+ B*mkilj+C*mkikj)-C*mkikj;
mwx=(ki+kj+3)*mkilj + kj*mkijj
mwy=-(ki+kj+3)*mlikj - kj*mjijj
mwz=ki*mjilj - kj*mlijj

% 1. area i=0 j=0 => ki=0 ji=-1 li=1  jj=-1 kj=0
% lj=1

mvx=-( A*m00)
mvy=-( B*m00)
mvz=3*(A*m10+ B*m01+C*m00)-C*m00;
mwx=3*m01 
mwy=-3*m10
mwz=0

% Sub  in m00=a m10/a=xg m01/a=yg 1/Zg=Axg+Byg+C

La=[-aA -aB  a*(3/Zg-C)  3ayg -3axg 0] % OK


%  Next try m10
% i=1 j=0 => ki=1 ji=0 li=2 , kj=0 jj=-1 lj=1

mvx=-1*(A*m10 + B*m01 + C*m00)-( A*m10)
mvy=-0*(A*m2jj + B*m10 + C*m1jj)-( B*m10)
mvz=(1+0+3)*(A*m20+ B*m11+C*m10)-C*m10;
mwx=(1+0+3)*m11 + 0*m1jj
mwy=-(1+0+3)*m20 - 0*m0jj
mwz=1*m01 - 0*m2jj

% clean up

mvx=-1*(A*m10 + B*m01 + C*m00)-( A*m10)
mvy=-( B*m10)
mvz=4*(A*m20+ B*m11+C*m10)-C*m10;
mwx=4*m11 
mwy=-4*m20 
mwz=m01

% substitutions

mvx=-1*(a/Zg)-( A*a*xg)
mvy=-( B*a*xg)
mvz=4*(A*m20+ B*m11+C*a*xg)-C*a*xg;
mwx=4*m11 
mwy=-4*m20 
mwz=yg*a
