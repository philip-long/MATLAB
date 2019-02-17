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
## @deftypefn {Function File} {@var{retval} =} Inertia_matrix (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-07-25

function A = Inertia_matrix (q)

global MX3 MY3 MZ3 MX2 MY2 MZ2 MX1 MY1 MZ1
global XX3 XX2 YY3 YY2 ZZ3 ZZ2 XX1 YY1 ZZ1
global XY1 XY2 XY3 XZ1 XZ2 XZ3 YZ1 YZ2 YZ3 
global d1 d2 d3 M1 M2 M3
global IA1 IA2 IA3


th1=q(1);
th2=q(2);
th3=q(3);

C1 = cos(th1);
S1 = sin(th1);
C2 = cos(th2);
S2 = sin(th2);
C3 = cos(th3);
S3 = sin(th3);
AS13 = C3*MX3 - MY3*S3;
AS23 = C3*MY3 + MX3*S3;
AJ113 = C3*XX3 - S3*XY3;
AJ213 = C3*XY3 + S3*XX3;
AJ123 = C3*XY3 - S3*YY3;
AJ223 = C3*YY3 + S3*XY3;
AJ133 = C3*XZ3 - S3*YZ3;
AJ233 = C3*YZ3 + S3*XZ3;
AJA113 = AJ113*C3 - AJ123*S3;
AJA213 = AJ213*C3 - AJ223*S3;
AJA123 = AJ113*S3 + AJ123*C3;
AJA223 = AJ213*S3 + AJ223*C3;
PAS213 = AS23*d3;
PAS313 = MZ3*d3;
PAS223 = -AS13*d3;
JP112 = AJA113 + XX2;
JP212 = AJA213 - PAS213 + XY2;
JP312 = AJ133 - PAS313 + XZ2;
JP122 = AJA123 - PAS213 + XY2;
JP222 = AJA223 + M3*d3**2 - 2*PAS223 + YY2;
JP322 = AJ233 + YZ2;
JP332 = M3*d3**2 - 2*PAS223 + ZZ2 + ZZ3;
MSP12 = AS13 + M3*d3 + MX2;
MSP22 = AS23 + MY2;
MSP32 = MZ2 + MZ3;
MP2 = M2 + M3;
AS12 = C2*MSP12 - MSP22*S2;
AS22 = C2*MSP22 + MSP12*S2;
AJ112 = C2*JP112 - JP212*S2;
AJ212 = C2*JP212 + JP112*S2;
AJ122 = C2*JP122 - JP222*S2;
AJ222 = C2*JP222 + JP122*S2;
AJ132 = C2*JP312 - JP322*S2;
AJ232 = C2*JP322 + JP312*S2;
AJA112 = AJ112*C2 - AJ122*S2;
AJA212 = AJ212*C2 - AJ222*S2;
AJA122 = AJ112*S2 + AJ122*C2;
AJA222 = AJ212*S2 + AJ222*C2;
PAS212 = AS22*d2;
PAS312 = MSP32*d2;
PAS222 = -AS12*d2;
JP111 = AJA112 + XX1;
JP211 = AJA212 - PAS212 + XY1;
JP311 = AJ132 - PAS312 + XZ1;
JP121 = AJA122 - PAS212 + XY1;
JP221 = AJA222 + MP2*d2**2 - 2*PAS222 + YY1;
JP321 = AJ232 + YZ1;
JP331 = JP332 + MP2*d2**2 - 2*PAS222 + ZZ1;
MSP11 = AS12 + MP2*d2 + MX1;
MSP21 = AS22 + MY1;
MSP31 = MSP32 + MZ1;
MP1 = M1 + MP2;
Nc31 = C2*MSP12*d2 + JP332 - MSP22*S2*d2;
Nd32 = C3*MX3*d3 - MY3*S3*d3 + ZZ3;
Ed11 = -AS13*S2 - AS23*C2;
Ed21 = AS13*C2 - AS23*S2;
Nd11 = AJ133*C2 - AJ233*S2;
Nd21 = AJ133*S2 + AJ233*C2;
Nd31 = AS13*C2*d2 - AS23*S2*d2 + Nd32;
A(1,1) = IA1 + JP331;
A(2,1) = Nc31;
A(3,1) = Nd31;
A(2,2) = IA2 + JP332;
A(3,2) = Nd32;
A(3,3) = IA3 + ZZ3;

  for i=1:3
  for j=1:3
  if i~=j
	  A(i,j)=A(j,i)	;
  end
  endfor
  endfor

endfunction
