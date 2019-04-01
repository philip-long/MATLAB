% Copyright (C) 2016 Philip
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
% @deftypefn {Function File} {@var{retval} =} plot_robot (@var{input1}, @var{input2})
%
% @seealso{}
% @end deftypefn

% Author: Philip <philip@philip-HP-ProBook-450-G1>
% Created: 2016-07-25

function [retval] = plot_robot (T1,T2,T3)

  plot(T1(1,4),T1(2,4),'bo',"MarkerSize",10.0,"LineWidth",4.0)
  hold on
  plot(T2(1,4),T2(2,4),'bo',"MarkerSize",10.0,"LineWidth",4.0)
  plot(T3(1,4),T3(2,4),'bo',"MarkerSize",10.0,"LineWidth",4.0)
  
  
  plot([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],'g',"LineWidth",4.0)
  plot([T2(1,4),T3(1,4)],[T2(2,4),T3(2,4)],'g',"LineWidth",4.0)
  

end