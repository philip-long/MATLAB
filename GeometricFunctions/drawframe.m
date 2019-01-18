% Copyright (C) 2015 philip
% 
% This program is free software; you can redistribute it and/or modify it
% under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% -*- texinfo -*- 
% @deftypefn {Function File} {@var{retval} =} drawframe (@var{input1}, @var{input2})
%
% @seealso{}
% @end deftypefn

% Author: philip <philip@philip-HP-ProBook-450-G1>
% Created: 2015-06-11

function [] = drawframe(r,scale)
%DRAWFRAME  plots a graphical description of a coordinate frame
%
%	DRAWFRAME(T)
%	DRAWFRAME(T, SCALE)
%
% H is a homogeneous transformation.  R is a rotation matrix, P is a point.
%
% See also: DRAWFRAMEDIFF, DRAWFRAMETRAJ, ANIMATEFRAMETRAJ.
 %Brad Kratochvil

% scale=0.005; 

  p = r(1:3,4);
  r = r(1:3,1:3);

  plot3(p(1), p(2), p(3));
  
  hchek = ishold;
  hold on;
  p=p';
  
plot3(p(1),p(2),p(3),'ko') % plot origin of the frame
  
  
hold on
Xaxisr=scale*r(1:3,1);
Yaxisr=scale*r(1:3,2);
Zaxisr=scale*r(1:3,3);
% plot Xaxis
plot3([p(1),p(1)+Xaxisr(1)],[p(2),p(2)+Xaxisr(2)],[p(3),p(3)+Xaxisr(3)],'r');
plot3([p(1),p(1)+Yaxisr(1)],[p(2),p(2)+Yaxisr(2)],[p(3),p(3)+Yaxisr(3)],'b');
plot3([p(1),p(1)+Zaxisr(1)],[p(2),p(2)+Zaxisr(2)],[p(3),p(3)+Zaxisr(3)],'g'); 

end
