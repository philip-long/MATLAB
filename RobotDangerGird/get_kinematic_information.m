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
## @deftypefn {Function File} {@var{retval} =} get_kinematic_information (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-12-05

function [Discrete_Pos,Positions,Jac ]=get_kinematic_information(q,qdot)

T_01=T01(q);T_02=T02(q);T_03=T03(q);
J_01=J01(q);J_02=J02(q);J_03=J03(q);
V=J_03*qdot;
A=Inertia_matrix(q);
M_inv=J_03(1:2,:)*inv(A)*transpose(J_03(1:2,:));
T12=inv(T_01)*T_02;
P12=T12(1:3,4);
T23=(inv(T_02)*T_03);
P23=T23(1:3,4);
Discrete_Pos{1}= P12;
Discrete_Pos{2}= P23;
Discrete_Pos{3}= [0;0;0];
Positions{1}=T_01;
Positions{2}=T_02;
Positions{3}=T_03;
Jac{1}=J_01;
Jac{2}=J_02;
Jac{3}=J_03;

endfunction
