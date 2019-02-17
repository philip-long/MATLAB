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
## @deftypefn {Function File} {@var{retval} =} T03 (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-07-25



function T = T03 (q)
global d2 d3

th1=q(1);
th2=q(2);
th3=q(3);
T0T3=eye(4);

T0T3(1,1) = cos(th1 + th2 + th3);
T0T3(2,1) = sin(th1 + th2 + th3);
T0T3(3,1) = 0;
T0T3(1,2) = -sin(th1 + th2 + th3);
T0T3(2,2) = cos(th1 + th2 + th3);
T0T3(3,2) = 0;
T0T3(1,3) = 0;
T0T3(2,3) = 0;
T0T3(3,3 )= 1;
T0T3(1,4) = d2*cos(th1) + d3*cos(th1 + th2);
T0T3(2,4) = d2*sin(th1) + d3*sin(th1 + th2);
T0T3(3,4) = 0;

T=T0T3;

endfunction
