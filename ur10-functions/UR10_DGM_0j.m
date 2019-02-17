## Copyright (C) 2015 philip
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
## @deftypefn {Function File} {@var{retval} =} UR10_DGM_0j (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: philip <philip@philip-HP-ProBook-450-G1>
## Created: 2015-03-30

function [T0j,Rosw] = UR10_DGM_0j(q)
% Shoulder to base
global R t rq

for i=1:7
  T{i}=t{i}*R{i};
  Rosw{i}=[T{i}(1:3,4)'  Rot_to_Quaternion_conROS(T{i}(1:3,1:3))];
end;  
  
for i=1:6
  T{i}=T{i}*TransMat(q(i),rq{i},'T');
  Rosw{i}=[T{i}(1:3,4)'  Rot_to_Quaternion_conROS(T{i}(1:3,1:3))];
end


Tpre=eye(4);

for i=1:length(T)
  T0j{i}=Tpre*T{i};
  Tpre=T0j{i};
  Rosw{i}=[T0j{i}(1:3,4)'  Rot_to_Quaternion_conROS(T0j{i}(1:3,1:3))];
endfor


endfunction