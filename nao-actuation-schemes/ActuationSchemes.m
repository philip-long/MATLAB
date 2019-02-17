% This script enumerates all the possible actuation schemes
%taking into account number of joints and number of actuators
clear all
close all
clc
c = combnk([1 2 3 4 5 6 7 8 9 10],4);

%% Group Combinations
% In order to size, so if a group is a subset of another it belongs smaller
% more exclusive group, example U-Joint on qr belongs to qr rather than UR
%

%Define Groups
QR=[1 2 3 4 5]; %Right arm Joints
QL=[6 7 8 9 10];%Left arm Joints

SR=[3 4 5]; %Spherical Joints right
SL=[8 9 10];%Spherical Joints left

UR=[1 2]; %U-Joints right
UL=[6 7]; %U-Joints left

ParBas=[1 6]; %Contain two base joints that are always parallel

NURL=[1 2 6 7]; %Contains none of the base joints

MUR=[3 4]; %Contains the middle U joint of the Rarm
MUL=[8 9]; %Contains the middle U joint of the Larm


R=zeros(size(c,1),size(c,2));L=zeros(size(c,1),size(c,2));Sr=zeros(size(c,1),size(c,2));
Sl=zeros(size(c,1),size(c,2));Ur=zeros(size(c,1),size(c,2));Ul=zeros(size(c,1),size(c,2));
NUR=zeros(size(c,1),size(c,2)); NUL=zeros(size(c,1),size(c,2));nobase=zeros(size(c,1),size(c,2));
parabas=zeros(size(c,1),size(c,2));mur=zeros(size(c,1),size(c,2));mul=zeros(size(c,1),size(c,2));
SymScheme=zeros(size(c,1),size(c,2));
NonAttributed=zeros(size(c,1),size(c,2));
spread=zeros(size(c,1),size(c,2));
%NonAttributed=0;

i=1;
for i=1:size(c,1)
    
    if all(ismember(c(i,:),QR)) % Check if all joints are contained in one arm
        R(i,:)=c(i,:);
    elseif all(ismember(c(i,:),QL)) % Check if all joints are contained in one arm
        L(i,:)=c(i,:);
    elseif  (c(i,3)-c(i,1)==5) &&  (c(i,4)-c(i,2)==5)
        SymScheme(i,:)=c(i,:);        
    elseif all(ismember(SR,c(i,:))) % Check if right S-joint is in c
        Sr(i,:)=c(i,:);
    elseif all(ismember(SL,c(i,:))) % Check if left S-joint is in c
        Sl(i,:)=c(i,:);
    elseif all(ismember(UR,c(i,:))) % Check if Right U-joint is in c
        Ur(i,:)=c(i,:);
    elseif all(ismember(UL,c(i,:))) % Check if left U-joint is in c
        Ul(i,:)=c(i,:);
    elseif ~any(ismember(NURL,c(i,:))) %Check if C does not contain any part of U joint from Left arm
        nobase(i,:)=c(i,:);
    elseif all(ismember(ParBas,c(i,:))) %Check if C does not contain any part of U joint from Left arm
        parabas(i,:)=c(i,:);
    elseif all(ismember(MUR,c(i,:))) % Check if right S-joint is in c
        mur(i,:)=c(i,:);
    elseif all(ismember(MUL,c(i,:))) % Check if left S-joint is in c
        mul(i,:)=c(i,:);
    elseif ~any(ismember(UR,c(i,:))) %Check if C does not contain any part of U joint from Right arm
        NUR(i,:)=c(i,:);
    elseif ~any(ismember(UL,c(i,:))) %Check if C does not contain any part of U joint from Left arm
        NUL(i,:)=c(i,:);
    elseif (nnz(ismember(c(i,:),QR)))==2 % split evenly across two arms
        spread(i,:)=c(i,:);
    else
        %NonAttributed = NonAttributed+1;
        NonAttributed(i,:)=c(i,:);
    end
    
end


%% Eliminate rows that are zero
R(~any(R,2),:) = [];
L(~any(L,2),:) = [];

SymScheme(~any(SymScheme,2),:)= [];

Sr(~any(Sr,2),:) = [];
Sl(~any(Sl,2),:) = [];

Ur(~any(Ur,2),:) = [];
Ul(~any(Ul,2),:) = [];

nobase(~any(nobase,2),:) = [];

mur(~any(mur,2),:)= [];
mul(~any(mul,2),:)= [];

parabas(~any(parabas,2),:) = [];

NUR(~any(NUR,2),:) = [];
NUL(~any(NUL,2),:) = [];

spread(~any(spread,2),:) = [];

NonAttributed(~any(NonAttributed,2),:) = [];

%% tetst

c=[1 3 7 10]


%Two on each arm
















