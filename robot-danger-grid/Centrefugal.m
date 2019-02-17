## Copyright (C) 2016 Philip
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {Function File} {@var{retval} =} Centrefugal (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-08-02

function [H] = Centrefugal (q,qdot)


global MX3 MY3 MZ3 MX2 MY2 MZ2 MX1 MY1 MZ1
global XX3 XX2 YY3 YY2 ZZ3 ZZ2 XX1 YY1 ZZ1
global XY1 XY2 XY3 XZ1 XZ2 XZ3 YZ1 YZ2 YZ3 
global d1 d2 d3 M1 M2 M3
global IA1 IA2 IA3 G3
th1=q(1);
th2=q(2);
th3=q(3);
QP1=qdot(1);
QP2=qdot(2);
QP3=qdot(3);

C1 = cos(th1);
S1 = sin(th1);
C2 = cos(th2);
S2 = sin(th2);
C3 = cos(th3);
S3 = sin(th3);
DV61 = QP1**2;
W32 = QP1 + QP2;
DV62 = W32**2;
VSP12 = -DV61*d2;
VP12 = C2*VSP12;
VP22 = -S2*VSP12;
W33 = QP3 + W32;
DV63 = W33**2;
VSP13 = -DV62*d3 + VP12;
VP13 = C3*VSP13 + S3*VP22;
VP23 = C3*VP22 - S3*VSP13;
F11 = -DV61*MX1;
F21 = -DV61*MY1;
F31 = -G3*M1;
PSI11 = QP1*XZ1;
PSI21 = QP1*YZ1;
PSI31 = QP1*ZZ1;
No11 = -PSI21*QP1;
No21 = PSI11*QP1;
F12 = -DV62*MX2 + M2*VP12;
F22 = -DV62*MY2 + M2*VP22;
F32 = -G3*M2;
PSI12 = W32*XZ2;
PSI22 = W32*YZ2;
PSI32 = W32*ZZ2;
No12 = -PSI22*W32;
No22 = PSI12*W32;
F13 = -DV63*MX3 + M3*VP13;
F23 = -DV63*MY3 + M3*VP23;
F33 = -G3*M3;
PSI13 = W33*XZ3;
PSI23 = W33*YZ3;
PSI33 = W33*ZZ3;
No13 = -PSI23*W33;
No23 = PSI13*W33;
N13 = -G3*MY3 - MZ3*VP23 + No13;
N23 = G3*MX3 + MZ3*VP13 + No23;
N33 = MX3*VP23 - MY3*VP13;
FDI13 = C3*F13 - F23*S3;
FDI23 = C3*F23 + F13*S3;
E12 = F12 + FDI13;
E22 = F22 + FDI23;
E32 = F32 + F33;
N12 = C3*N13 - G3*MY2 - MZ2*VP22 - N23*S3 + No12;
N22 = C3*N23 - F33*d3 + G3*MX2 + MZ2*VP12 + N13*S3 + No22;
N32 = FDI23*d3 + MX2*VP22 - MY2*VP12 + N33;
FDI12 = C2*E12 - E22*S2;
FDI22 = C2*E22 + E12*S2;
E11 = F11 + FDI12;
E21 = F21 + FDI22;
E31 = E32 + F31;
N11 = C2*N12 - G3*MY1 - N22*S2 + No11;
N21 = C2*N22 - E32*d2 + G3*MX1 + N12*S2 + No21;
N31 = FDI22*d2 + N32;
FDI11 = C1*E11 - E21*S1;
FDI21 = C1*E21 + E11*S1;
GAM1 = N31;
GAM2 = N32;
GAM3 = N33;
H=[GAM1;GAM2;GAM3];
endfunction
