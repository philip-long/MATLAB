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
## @deftypefn {Function File} {@var{retval} =} J20 (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-07-25


function J= J02 (q)
global d2
th1=q(1);
th2=q(2);

J=zeros(6,2);

J(1,1) = -d2*sin(th1);
J(2,1) = d2*cos(th1);
J(3,1) = 0;
J(4,1) = 0;
J(5,1) = 0;
J(6,1) = 1;
J(1,2) = 0;
J(2,2) = 0;
J(3,2) = 0;
J(4,2) = 0;
J(5,2) = 0;
J(6,2) = 1;

J=J([1,2,6],:);

endfunction
