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
## @deftypefn {Function File} {@var{retval} =} UR10_DGM (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: philip <philip@philip-HP-ProBook-450-G1>
## Created: 2015-03-27

function [T,Ros] = UR10_DGM (q)

% Shoulder to base

R{1}=ZYX_to_Rot([0.0 0.0 0.0],"T");
t{1}=TransMat([0.0 0.0 0.1273]);
%rq{1}=[0 ;0 ;1];
rq{1}="z";

% upper_arm_link to shoulder_link
R{2}=ZYX_to_Rot([0.0 1.570796325 0.0],"T");
quaternion2matrix_conROS([0.0 0.707 0.0 0.707]);
t{2}=TransMat([0.0 0.220941 0.0]);
%rq{2}=[0 ;1 ;0];
rq{2}="y";

% forearm_link to upper_arm_link
R{3}=ZYX_to_Rot([0.0 0.0 0.0],"T");
t{3}=TransMat([0.0 -0.1719 0.612]);
%rq{3}=[0 ;1 ;0];
rq{3}="y";

% wrist_1_link to forearm_link
R{4}=ZYX_to_Rot([0.0 1.570796325 0.0],"T");
t{4}=TransMat([0.0 0.0 0.5723]);
%rq{4}=[0 ;1 ;0];
rq{4}="y";

% wrist_2_link to wrist_1_link
R{5}=ZYX_to_Rot([0.0 0.0 0.0],"T");
t{5}=TransMat([0.0 0.1149 0.0]);
rq{5}=[0 ;0; 1];
rq{5}="z";

% wrist_3_link to wrist_2_link
R{6}=ZYX_to_Rot([0.0 0.0 0.0],"T");
t{6}=TransMat([0.0 0.0 0.1157]);
r{6}=[0 ;1 ;0];
rq{6}="y";

% ee_link to wrist_3_link
R{7}=ZYX_to_Rot([0.0 0.0 1.570796325],"T");
quaternion2matrix_conROS([0.0 0.0 0.707 0.707]);
t{7}=TransMat([0.0 0.0922 0.0]);

for i=1:7
  T{i}=t{i}*R{i};
  Ros{i}=[T{i}(1:3,4)'  Rot_to_Quaternion_conROS(T{i}(1:3,1:3))];
  RosRPY{i}=[T{i}(1:3,4) ; Rot_to_ZYX(T{i}(1:3,1:3))]';
end;  
  
for i=1:6
  T{i}=T{i}*TransMat(q(i),rq{i},'T');
  Ros{i}=[T{i}(1:3,4)'  Rot_to_Quaternion_conROS(T{i}(1:3,1:3))];
  RosRPY{i}=[T{i}(1:3,4) ; Rot_to_ZYX(T{i}(1:3,1:3))]';
end

endfunction
