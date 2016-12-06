clear all,clc
global r1 r4 r7 r5 d4 d3



r1=0.128;
r4=0.1639;
r7=0.0922;
r5=.1157;
d4=0.5716;
d3=0.6127;
re=0.1;
r7=r7+re;

%% D H Parameters

global alpha d theta r sigma Joints

Joints=6
ant = [0,1,2,3,4,5,6];
sigma = [0,0,0,0,0,0,2];
b = [0,0,0,0,0,0];
d = [0,0,-d3,-d4,0,0];
r = [r1,0,0.0,r4,r5,0];
gamma = [0,0,0,0,0,0];
alpha = [0,pi/2,0,0,pi/2,-pi/2,0];
mu = [1,1,1,1,1,1,0];
theta = [0,0,0,0,0,0,0];


% Dynamic Parameters

%global   XX1     XY1     XZ1     YY1     YZ1     ZZ1     MX1     MY1     MZ1     M1      IA1     
%global   XX2     XY2     XZ2     YY2     YZ2     ZZ2     MX2     MY2     MZ2     M2      IA2     
%global   XX3     XY3     XZ3     YY3     YZ3     ZZ3     MX3     MY3     MZ3     M3      IA3     
%global   XX4     XY4     XZ4     YY4     YZ4     ZZ4     MX4     MY4     MZ4     M4      IA4     
%global   XX5     XY5     XZ5     YY5     YZ5     ZZ5     MX5     MY5     MZ5     M5      IA5     
%global   XX6     XY6     XZ6     YY6     YZ6     ZZ6     MX6     MY6     MZ6     M6      IA6     
%global   XX7     XY7     XZ7     YY7     YZ7     ZZ7     MX7     MY7     MZ7     M7      IA7  

%global FS1     FV1   FX6     FY6     FZ6     CX6     CY6     CZ6     FS6     FV6     
%global FS2     FV2   FX7     FY7     FZ7     CX7     CY7     CZ7     FS7     FV7      
%global FS3     FV3           
%global FS4     FV4     
%global FS5     FV5 G3

  
%Dynamic inertia parameters

%XX1=0.031474;   XY1=0.0;     XZ1=0.0; YY1=0.31474; YZ1=0.0;     ZZ1=  0.021876;    MX1=0.0;     MY1=0.0;     MZ1=0.0;   M1 =7.7780;  IA1=0.0;     
%XX2 =0.42175;     XY2=0.0;     XZ2=0.0;     YY2=0.42175;     YZ2=0.0;     ZZ2=  0.036366;     MX2=0.0;     MY2=0.0;     MZ2=3.95658;       M2=  12.930;      IA2=0.0;     
%XX3=0.11107;      XY3=0.0;     XZ3 =0.0;    YY3=0.11107;     YZ3=0.0;     ZZ3 =  0.010884;    MX3=0.0;     MY3=0.0;     MZ3=1.10740;  M3 =  3.8700;     IA3=0.0;     
%XX4=0.0051082;  XY4=0.0;    XZ4=0.0; YY4=0.0051082;  YZ4=0.0; ZZ4=  0.0055125; MX4=0.0;     MY4=0.0;     MZ4=0.0; M4=1.960;   IA4=0.0;     
%XX5=0.0051082;    XY5=0.0;   XZ5=0.0;     YY5=0.0051082;     YZ5=0.0;     ZZ5   =  0.0055125;   MX5=0.0;     MY5=0.0;     MZ5=0.0;     M5=1.9600;    IA5=0.0;     
%XX6=5.2646e-04;   XY6=0.0;  XZ6=0.0;     YY6=5.2646e-04;     YZ6=0.0;     ZZ6  =    5.6812e-04;   MX6=0.0;     MY6=0.0;     MZ6=0.0;     M6=0.20200;      IA6=0.0;     
%XX7=0.0;     XY7=0.0;     XZ7=0.0;     YY7=0.0;     YZ7=0.0;     ZZ7=0.0;     MX7=0.0;     MY7=0.0;     MZ7=0.0;     M7=3.45;      IA7=0.0;     

%External forces and joint parameters
%j       FX      FY      FZ      CX      CY      CZ      FS      FV      QP      QDP     GAM     eta     k       
%FS1=0.0;     FV1=0.0;   
%FS2=0.0;     FV2=0.0;     
%FS3=0.0;     FV3=0.0;           
%FS4=0.0;     FV4=0.0;     
%FS5=0.0;     FV5=0.0;     

%FX6=0.0;FY6=0.0;FZ6=0.0;CX6=0.0;CY6=0.0;CZ6=0.0;FS6=0.0;FV6=0.0;
%FX7=0.0;FY7=0.0;FZ7=0.0;CX7=0.0;CY7=0.0;CZ7=0.0;FS7=0.0;FV7=0.0;  
G3=-9.8066;
%6       FX6     FY6     FZ6     CX6     CY6     CZ6     FS6     FV6     QP6     QDP6    GAM6    0       0       
%7       FX7     FY7     FZ7     CX7     CY7     CZ7     FS7     FV7     0       0       0       0       0       

%Base velicities parameters
%axis    W0      WP0     V0      VP0     G       
%X       0       0       0       0       0       
%Y       0       0       0       0       0       
%Z       0       0       0       0       G3     

global XX XY XZ YZ YY ZZ M MX MY MZ G3 I_j MS
% Extract all the inertia variables for the robot
G3=-9.8066;

for link=1:7
	[I_j{link},M{link},MS{link}]=CalculateInertiaUR10(link);
	XX{link}=I_j{link}(1,1);
	XY{link}=I_j{link}(1,2);
	XZ{link}=I_j{link}(1,3);
	YZ{link}=I_j{link}(2,3);
	YY{link}=I_j{link}(2,2);
	ZZ{link}=I_j{link}(3,3);
	MX{link}=MS{link}(1);
	MY{link}=MS{link}(2);
	MZ{link}=MS{link}(3);	
end


