% This script finds the parameters need to calculate the reoloved force
% online at the tool frame, by formulated a lesst squares optimization
% problem
%
%  Two possible methods
% 
%  (a) Inputs:  bMe,eMf,eMt
% (a) Using the end effector location and the known 
% transformation matrices between the end effector the tool and force
% sensor
%
%  (b) Inputs:  bMt,tMf
%(b) Using the tool location data and a known transformation matrix between
% tool and force sensor
%
%
%  Ouputs: Parameters for ForceResolutionatToolFrame.m or
%  ZeroForceSensor.cpp

Force = importdata('Force.txt');
bMe_new = importdata('bMe.txt');

j=1;
for i=1:4:(length(bMe_new))
    bMe(1:4,:,j)=bMe_new(i:i+3,:);
    j=j+1;
end

%X =[ b1, b2, b3, b4, b5, b6, M_fs, fPXcg_t, fPYcg_t, fPZcg_t]
 
% No values for eMf and eMt therefore estimate, normally well known

eMf=TransMat(pi/4,'rz','T')*TransMat([0 0 0.05],'T')
fMt=[0.0 1 0 0;-1 0 0 0; 0 0 1 0.14; 0 0 0 1];
eMt=eMf*fMt;
% (a)

tMf=(eMt)\eMf;

% (b)
%fMb=inv(bMt*tMf);




% -----------------------------ASIGN THE VARIABLES-------------------------


tMf11=tMf(1,1);tMf12=tMf(1,2);tMf13=tMf(1,3);
tMf21=tMf(2,1);tMf22=tMf(2,2);tMf23=tMf(1,3);
tMf31=tMf(3,1);tMf32=tMf(3,2);tMf33=tMf(3,3);


M_t=1;
fPxt=fMt(1,4);
fPyt=fMt(2,4);
fPzt=fMt(3,4);
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3);

Y_Measure=[];
W_observe=[];
i=1
for i=1:length(Force)
    
    % Find variables
    % Find measured force in newtons and newton metres
    F=Force(i,:)/1000000;
    fMb=inv(bMe(:,:,1)*eMf);
    
    fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
    fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(1,3);
    fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);


    

    % Transform measured force to tool frame
    tF=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*F';
    
    
    % W is found from Resolution Procedure
W =[     -tMf11,                  -tMf12,                  -tMf13,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
         -tMf21,                  -tMf22,                  -tMf23,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
                  -tMf31,                  -tMf32,                  -tMf33,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
 fPzt*tMf21 - fPyt*tMf31, fPzt*tMf22 - fPyt*tMf32, fPzt*tMf23 - fPyt*tMf33, -tMf11, -tMf12, -tMf13, 0, (981*M_t*fMb33*tMf12)/100 - (981*M_t*fMb23*tMf13)/100, (981*M_t*fMb13*tMf13)/100 - (981*M_t*fMb33*tMf11)/100, (981*M_t*fMb23*tMf11)/100 - (981*M_t*fMb13*tMf12)/100
 fPxt*tMf31 - fPzt*tMf11, fPxt*tMf32 - fPzt*tMf12, fPxt*tMf33 - fPzt*tMf13, -tMf21, -tMf22, -tMf23, 0, (981*M_t*fMb33*tMf22)/100 - (981*M_t*fMb23*tMf23)/100, (981*M_t*fMb13*tMf23)/100 - (981*M_t*fMb33*tMf21)/100, (981*M_t*fMb23*tMf21)/100 - (981*M_t*fMb13*tMf22)/100
 fPyt*tMf11 - fPxt*tMf21, fPyt*tMf12 - fPxt*tMf22, fPyt*tMf13 - fPxt*tMf23, -tMf31, -tMf32, -tMf33, 0, (981*M_t*fMb33*tMf32)/100 - (981*M_t*fMb23*tMf33)/100, (981*M_t*fMb13*tMf33)/100 - (981*M_t*fMb33*tMf31)/100, (981*M_t*fMb23*tMf31)/100 - (981*M_t*fMb13*tMf32)/100];

Y_Measure=[Y_Measure; tF];
    W_observe=[W_observe;W];
    i
end

x=pinv(W_observe)*Y_MeasureW


%% Check the answer, to see if it cancels the forces at tool frame
Force = importdata('Force.txt');
bMe_new = importdata('bMe.txt');
j=1;
for i=1:4:(length(bMe_new))
    bMe(1:4,:,j)=bMe_new(i:i+3,:);
    j=j+1;
end

for i=1:length(Force)
eMf=TransMat(pi/4,'rz','T')*TransMat([0 0 0.05],'T');
fMt=[0.0 1 0 0;-1 0 0 0; 0 0 1 0.14; 0 0 0 1];
eMt=eMf*fMt;
tMf=(eMt)\eMf;
F=Force(i,:)/1000000;
fMb=inv(bMe(:,:,1)*eMf);
fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(1,3);
fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);

% Measured Force at tool frame
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3);
tFrev=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*F';
   
W =[     -tMf11,                  -tMf12,                  -tMf13,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
        -tMf21,                  -tMf22,                  -tMf23,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
        -tMf31,                  -tMf32,                  -tMf33,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
 fPzt*tMf21 - fPyt*tMf31, fPzt*tMf22 - fPyt*tMf32, fPzt*tMf23 - fPyt*tMf33, -tMf11, -tMf12, -tMf13, 0, (981*M_t*fMb33*tMf12)/100 - (981*M_t*fMb23*tMf13)/100, (981*M_t*fMb13*tMf13)/100 - (981*M_t*fMb33*tMf11)/100, (981*M_t*fMb23*tMf11)/100 - (981*M_t*fMb13*tMf12)/100
 fPxt*tMf31 - fPzt*tMf11, fPxt*tMf32 - fPzt*tMf12, fPxt*tMf33 - fPzt*tMf13, -tMf21, -tMf22, -tMf23, 0, (981*M_t*fMb33*tMf22)/100 - (981*M_t*fMb23*tMf23)/100, (981*M_t*fMb13*tMf23)/100 - (981*M_t*fMb33*tMf21)/100, (981*M_t*fMb23*tMf21)/100 - (981*M_t*fMb13*tMf22)/100
 fPyt*tMf11 - fPxt*tMf21, fPyt*tMf12 - fPxt*tMf22, fPyt*tMf13 - fPxt*tMf23, -tMf31, -tMf32, -tMf33, 0, (981*M_t*fMb33*tMf32)/100 - (981*M_t*fMb23*tMf33)/100, (981*M_t*fMb13*tMf33)/100 - (981*M_t*fMb33*tMf31)/100, (981*M_t*fMb23*tMf31)/100 - (981*M_t*fMb13*tMf32)/100];
tFrev;
Ft=W*x;
tFrev-Ft;
plot(i,(tFrev-Ft))
%plot(i,tFrev)
hold on
end