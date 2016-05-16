% Converting the ouput of the Catia model to usable data for the dynamic
% simulation

clear all
clc
% Position of Centre of Graivty of 7 links in World frame
%   0Gj

G=[ 0	23.94219	223.02785
    0	-25.88272	386.07729
    0	-25.94219	623.02785
    0	23.88272	786.07729
    0	21.56662	991.17064
    0	-4.01763	1097.54687
    0	0.5         1162.68533];

% Position of the Origin of the links in World frame
% 0Oj

 O=[0	0	310.5
    0	0	310.5
    0	0	710.5
    0	0	710.5
    0	0	1100.5
    0	0	1100.5
    0	0	1178.5];

% Mass 
% Mj
M=[2.58844
2.58844
2.58844
2.58844
1.77778
1.72146
0.12532];

%            R(1,1),R(1,2),R(1,3),R(2,1)
%
%   jR0

Orientations=[  1       0      0       0	1	0	0	0	1
                1       0       0      0	0	1	0	-1	0
                1       0       0      0	1	0	0	0	1
                1       0       0      0	0	-1	0	1	0
                1       0       0      0	1	0	0	0	1
                1       0       0      0	0	1	0	-1	0
                1       0       0      0	1	0	0	0	1];
R=cell(1,7)  ;       

% This gives jR0, 0Tj
for i=1:7
    jj=1; 
    for j=1:3:9
        R{i}(jj,:)=Orientations(i,j:j+2);
        jj=jj+1;
    end
    T{i}=[R{i}' O(i,:)';0 0 0 1];
end
            
            
% Firstly find MX MY MZ:
% T{i} is the transformation matrix from 0 to i
% G{i} is the centre of mass in the 0 frame

for i=1:7
    Mxyz(i,:)=(T{i}\([G(i,:) M(i)^-1])').*M(i);
end




