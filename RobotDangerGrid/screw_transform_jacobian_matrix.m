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
## @deftypefn {Function File} {@var{retval} =} screw_transform_jacobian_matrix (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Philip <philip@philip-HP-ProBook-450-G1>
## Created: 2016-12-05

% R is rotation from link to world frame
% p is distance from link origin to point on link in link frame
% Jin is the jacobian associated with the link origin

% return the Jacobian of point on link defined by vector p from link origin

function [J] = screw_transform_jacobian_matrix (R,p,Jin)


 L= R*(p); % Rotate the point to the world frame
 Lhat=skew(L);  %  
 ScrewReduced=[eye(3) -Lhat; zeros(3) eye(3) ];
 Screw2=ScrewReduced(:,[1,2,6]);            
 J=Screw2*Jin; % Change the point of the Jacobiansobian
 J=J([1,2,6],:); 
       
       
endfunction
