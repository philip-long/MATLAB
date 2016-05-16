%Inverse Dynamic Model using Newton-Euler Algorithm
%Robot with rigid joints and fixed base


%Geometric parameters
%j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
%1       0       0       1       0       0       0       0       th1     r1      
%2       1       0       1       0       0       pi/2    0       th2     0       
%3       2       0       1       0       0       0       -d3     th3     0.0     
%4       3       0       1       0       0       0       -d4     th4     r4      
%5       4       0       1       0       0       pi/2    0       th5     r5      
%6       5       0       1       0       0       -pi/2   0       th6     0       
%7       6       2       0       0       0       0       0       0       r7      

%Dynamic inertia parameters
%j       XX      XY      XZ      YY      YZ      ZZ      MX      MY      MZ      M       IA      
%1       XX1     XY1     XZ1     YY1     YZ1     ZZ1     MX1     MY1     MZ1     M1      IA1     
%2       XX2     XY2     XZ2     YY2     YZ2     ZZ2     MX2     MY2     MZ2     M2      IA2     
%3       XX3     XY3     XZ3     YY3     YZ3     ZZ3     MX3     MY3     MZ3     M3      IA3     
%4       XX4     XY4     XZ4     YY4     YZ4     ZZ4     MX4     MY4     MZ4     M4      IA4     
%5       XX5     XY5     XZ5     YY5     YZ5     ZZ5     MX5     MY5     MZ5     M5      IA5     
%6       XX6     XY6     XZ6     YY6     YZ6     ZZ6     MX6     MY6     MZ6     M6      IA6     
%7       XX7     XY7     XZ7     YY7     YZ7     ZZ7     MX7     MY7     MZ7     M7      IA7     

%External forces and joint parameters
%j       FX      FY      FZ      CX      CY      CZ      FS      FV      QP      QDP     GAM     eta     k       
%1       0       0       0       0       0       0       FS1     FV1     QP1     QDP1    GAM1    0       0       
%2       0       0       0       0       0       0       FS2     FV2     QP2     QDP2    GAM2    0       0       
%3       0       0       0       0       0       0       FS3     FV3     QP3     QDP3    GAM3    0       0       
%4       0       0       0       0       0       0       FS4     FV4     QP4     QDP4    GAM4    0       0       
%5       0       0       0       0       0       0       FS5     FV5     QP5     QDP5    GAM5    0       0       
%6       FX6     FY6     FZ6     CX6     CY6     CZ6     FS6     FV6     QP6     QDP6    GAM6    0       0       
%7       FX7     FY7     FZ7     CX7     CY7     CZ7     FS7     FV7     0       0       0       0       0       

%Base velicities parameters
%axis    W0      WP0     V0      VP0     G       
%X       0       0       0       0       0       
%Y       0       0       0       0       0       
%Z       0       0       0       0       G3      

function GAM=ur10_e_idm(u)

global   XX1     XY1     XZ1     YY1     YZ1     ZZ1     MX1     MY1     MZ1     M1      IA1     
global   XX2     XY2     XZ2     YY2     YZ2     ZZ2     MX2     MY2     MZ2     M2      IA2     
global   XX3     XY3     XZ3     YY3     YZ3     ZZ3     MX3     MY3     MZ3     M3      IA3     
global   XX4     XY4     XZ4     YY4     YZ4     ZZ4     MX4     MY4     MZ4     M4      IA4     
global   XX5     XY5     XZ5     YY5     YZ5     ZZ5     MX5     MY5     MZ5     M5      IA5     
global   XX6     XY6     XZ6     YY6     YZ6     ZZ6     MX6     MY6     MZ6     M6      IA6     
global   XX7     XY7     XZ7     YY7     YZ7     ZZ7     MX7     MY7     MZ7     M7      IA7  

global FS1     FV1   FX6     FY6     FZ6     CX6     CY6     CZ6     FS6     FV6     
global FS2     FV2   FX7     FY7     FZ7     CX7     CY7     CZ7     FS7     FV7      
global FS3     FV3           
global FS4     FV4     
global FS5     FV5   

global r1 r4 r7 r5 d4 d3 G3

th1=u(1);
th2=u(2);
th3=u(3);
th4=u(4);
th5=u(5);
th6=u(6);
QP1=u(7);
QP2=u(8);
QP3=u(9);
QP4=u(10);
QP5=u(11);
QP6=u(12);
QDP1=u(13);
QDP2=u(14);
QDP3=u(15);
QDP4=u(16);
QDP5=u(17);
QDP6=u(18);


C1 = cos(th1);
S1 = sin(th1);
C2 = cos(th2);
S2 = sin(th2);
C3 = cos(th3);
S3 = sin(th3);
C4 = cos(th4);
S4 = sin(th4);
C5 = cos(th5);
S5 = sin(th5);
C6 = cos(th6);
S6 = sin(th6);
DV61 = QP1**2;
W12 = QP1*S2;
W22 = C2*QP1;
WP12 = QDP1*S2 + QP2*W22;
WP22 = C2*QDP1 - QP2*W12;
DV12 = W12**2;
DV22 = W12*W22;
DV32 = QP2*W12;
DV42 = W22**2;
DV52 = QP2*W22;
DV62 = QP2**2;
U112 = -DV42 - DV62;
U212 = DV22 + QDP2;
U312 = DV32 - WP22;
U122 = DV22 - QDP2;
U222 = -DV12 - DV62;
U322 = DV52 + WP12;
U132 = DV32 + WP22;
U232 = DV52 - WP12;
U332 = -DV12 - DV42;
VP12 = -G3*S2;
VP22 = -C2*G3;
W13 = C3*W12 + S3*W22;
W23 = C3*W22 - S3*W12;
W33 = QP2 + QP3;
WP13 = C3*WP12 + QP3*W23 + S3*WP22;
WP23 = C3*WP22 - QP3*W13 - S3*WP12;
WP33 = QDP2 + QDP3;
DV13 = W13**2;
DV23 = W13*W23;
DV33 = W13*W33;
DV43 = W23**2;
DV53 = W23*W33;
DV63 = W33**2;
U113 = -DV43 - DV63;
U213 = DV23 + WP33;
U313 = DV33 - WP23;
U123 = DV23 - WP33;
U223 = -DV13 - DV63;
U323 = DV53 + WP13;
U133 = DV33 + WP23;
U233 = DV53 - WP13;
U333 = -DV13 - DV43;
VSP13 = -U112*d3 + VP12;
VSP23 = -U212*d3 + VP22;
VSP33 = -U312*d3;
VP13 = C3*VSP13 + S3*VSP23;
VP23 = C3*VSP23 - S3*VSP13;
W14 = C4*W13 + S4*W23;
W24 = C4*W23 - S4*W13;
W34 = QP4 + W33;
WP14 = C4*WP13 + QP4*W24 + S4*WP23;
WP24 = C4*WP23 - QP4*W14 - S4*WP13;
WP34 = QDP4 + WP33;
DV14 = W14**2;
DV24 = W14*W24;
DV34 = W14*W34;
DV44 = W24**2;
DV54 = W24*W34;
DV64 = W34**2;
U114 = -DV44 - DV64;
U214 = DV24 + WP34;
U314 = DV34 - WP24;
U124 = DV24 - WP34;
U224 = -DV14 - DV64;
U324 = DV54 + WP14;
U134 = DV34 + WP24;
U234 = DV54 - WP14;
U334 = -DV14 - DV44;
VSP14 = -U113*d4 + U133*r4 + VP13;
VSP24 = -U213*d4 + U233*r4 + VP23;
VSP34 = -U313*d4 + U333*r4 + VSP33;
VP14 = C4*VSP14 + S4*VSP24;

VP24 = C4*VSP24 - S4*VSP14;
W15 = C5*W14 + S5*W34;
W25 = C5*W34 - S5*W14;
W35 = QP5 - W24;
WP15 = C5*WP14 + QP5*W25 + S5*WP34;
WP25 = C5*WP34 - QP5*W15 - S5*WP14;
WP35 = QDP5 - WP24;
DV15 = W15**2;
DV25 = W15*W25;
DV35 = W15*W35;
DV45 = W25**2;
DV55 = W25*W35;
DV65 = W35**2;
U115 = -DV45 - DV65;
U215 = DV25 + WP35;
U315 = DV35 - WP25;
U125 = DV25 - WP35;
U225 = -DV15 - DV65;
U325 = DV55 + WP15;
U135 = DV35 + WP25;
U235 = DV55 - WP15;
U335 = -DV15 - DV45;
VSP15 = -U124*r5 + VP14;
VSP25 = -U224*r5 + VP24;
VSP35 = -U324*r5 + VSP34;
VP15 = C5*VSP15 + S5*VSP35;
VP25 = C5*VSP35 - S5*VSP15;
W16 = C6*W15 - S6*W35;
W26 = -C6*W35 - S6*W15;
W36 = QP6 + W25;
WP16 = C6*WP15 + QP6*W26 - S6*WP35;
WP26 = -C6*WP35 - QP6*W16 - S6*WP15;
WP36 = QDP6 + WP25;
DV16 = W16**2;
DV26 = W16*W26;
DV36 = W16*W36;
DV46 = W26**2;
DV56 = W26*W36;
DV66 = W36**2;
U116 = -DV46 - DV66;
U216 = DV26 + WP36;
U316 = DV36 - WP26;
U126 = DV26 - WP36;
U226 = -DV16 - DV66;
U326 = DV56 + WP16;
U136 = DV36 + WP26;
U236 = DV56 - WP16;
U336 = -DV16 - DV46;
VP16 = C6*VP15 + S6*VSP25;
VP26 = C6*VSP25 - S6*VP15;
VSP17 = U136*r7 + VP16;
VSP27 = U236*r7 + VP26;
VSP37 = U336*r7 + VP25;
F11 = -DV61*MX1 - MY1*QDP1;
F21 = -DV61*MY1 + MX1*QDP1;
F31 = -G3*M1;
PSI11 = QP1*XZ1;
PSI21 = QP1*YZ1;
PSI31 = QP1*ZZ1;
No11 = -PSI21*QP1 + QDP1*XZ1;
No21 = PSI11*QP1 + QDP1*YZ1;
No31 = QDP1*ZZ1;
F12 = M2*VP12 + MX2*U112 + MY2*U122 + MZ2*U132;
F22 = M2*VP22 + MX2*U212 + MY2*U222 + MZ2*U232;
F32 = MX2*U312 + MY2*U322 + MZ2*U332;
PSI12 = QP2*XZ2 + W12*XX2 + W22*XY2;
PSI22 = QP2*YZ2 + W12*XY2 + W22*YY2;
PSI32 = QP2*ZZ2 + W12*XZ2 + W22*YZ2;
No12 = -PSI22*QP2 + PSI32*W22 + QDP2*XZ2 + WP12*XX2 + WP22*XY2;
No22 = PSI12*QP2 - PSI32*W12 + QDP2*YZ2 + WP12*XY2 + WP22*YY2;
No32 = -PSI12*W22 + PSI22*W12 + QDP2*ZZ2 + WP12*XZ2 + WP22*YZ2;
F13 = M3*VP13 + MX3*U113 + MY3*U123 + MZ3*U133;
F23 = M3*VP23 + MX3*U213 + MY3*U223 + MZ3*U233;
F33 = M3*VSP33 + MX3*U313 + MY3*U323 + MZ3*U333;
PSI13 = W13*XX3 + W23*XY3 + W33*XZ3;
PSI23 = W13*XY3 + W23*YY3 + W33*YZ3;
PSI33 = W13*XZ3 + W23*YZ3 + W33*ZZ3;
No13 = -PSI23*W33 + PSI33*W23 + WP13*XX3 + WP23*XY3 + WP33*XZ3;
No23 = PSI13*W33 - PSI33*W13 + WP13*XY3 + WP23*YY3 + WP33*YZ3;
No33 = -PSI13*W23 + PSI23*W13 + WP13*XZ3 + WP23*YZ3 + WP33*ZZ3;
F14 = M4*VP14 + MX4*U114 + MY4*U124 + MZ4*U134;
F24 = M4*VP24 + MX4*U214 + MY4*U224 + MZ4*U234;
F34 = M4*VSP34 + MX4*U314 + MY4*U324 + MZ4*U334;
PSI14 = W14*XX4 + W24*XY4 + W34*XZ4;
PSI24 = W14*XY4 + W24*YY4 + W34*YZ4;
PSI34 = W14*XZ4 + W24*YZ4 + W34*ZZ4;
No14 = -PSI24*W34 + PSI34*W24 + WP14*XX4 + WP24*XY4 + WP34*XZ4;
No24 = PSI14*W34 - PSI34*W14 + WP14*XY4 + WP24*YY4 + WP34*YZ4;
No34 = -PSI14*W24 + PSI24*W14 + WP14*XZ4 + WP24*YZ4 + WP34*ZZ4;
F15 = M5*VP15 + MX5*U115 + MY5*U125 + MZ5*U135;
F25 = M5*VP25 + MX5*U215 + MY5*U225 + MZ5*U235;
F35 = -M5*VSP25 + MX5*U315 + MY5*U325 + MZ5*U335;
PSI15 = W15*XX5 + W25*XY5 + W35*XZ5;
PSI25 = W15*XY5 + W25*YY5 + W35*YZ5;
PSI35 = W15*XZ5 + W25*YZ5 + W35*ZZ5;
No15 = -PSI25*W35 + PSI35*W25 + WP15*XX5 + WP25*XY5 + WP35*XZ5;
No25 = PSI15*W35 - PSI35*W15 + WP15*XY5 + WP25*YY5 + WP35*YZ5;
No35 = -PSI15*W25 + PSI25*W15 + WP15*XZ5 + WP25*YZ5 + WP35*ZZ5;
F16 = M6*VP16 + MX6*U116 + MY6*U126 + MZ6*U136;
F26 = M6*VP26 + MX6*U216 + MY6*U226 + MZ6*U236;
F36 = M6*VP25 + MX6*U316 + MY6*U326 + MZ6*U336;
PSI16 = W16*XX6 + W26*XY6 + W36*XZ6;
PSI26 = W16*XY6 + W26*YY6 + W36*YZ6;
PSI36 = W16*XZ6 + W26*YZ6 + W36*ZZ6;
No16 = -PSI26*W36 + PSI36*W26 + WP16*XX6 + WP26*XY6 + WP36*XZ6;
No26 = PSI16*W36 - PSI36*W16 + WP16*XY6 + WP26*YY6 + WP36*YZ6;
No36 = -PSI16*W26 + PSI26*W16 + WP16*XZ6 + WP26*YZ6 + WP36*ZZ6;
F17 = M7*VSP17 + MX7*U116 + MY7*U126 + MZ7*U136;
F27 = M7*VSP27 + MX7*U216 + MY7*U226 + MZ7*U236;
F37 = M7*VSP37 + MX7*U316 + MY7*U326 + MZ7*U336;
PSI17 = W16*XX7 + W26*XY7 + W36*XZ7;
PSI27 = W16*XY7 + W26*YY7 + W36*YZ7;
PSI37 = W16*XZ7 + W26*YZ7 + W36*ZZ7;
No17 = -PSI27*W36 + PSI37*W26 + WP16*XX7 + WP26*XY7 + WP36*XZ7;
No27 = PSI17*W36 - PSI37*W16 + WP16*XY7 + WP26*YY7 + WP36*YZ7;
No37 = -PSI17*W26 + PSI27*W16 + WP16*XZ7 + WP26*YZ7 + WP36*ZZ7;
E17 = F17 + FX7;
E27 = F27 + FY7;
E37 = F37 + FZ7;
N17 = CX7 + MY7*VSP37 - MZ7*VSP27 + No17;
N27 = CY7 - MX7*VSP37 + MZ7*VSP17 + No27;
N37 = CZ7 + MX7*VSP27 - MY7*VSP17 + No37;
E16 = E17 + F16 + FX6;
E26 = E27 + F26 + FY6;
E36 = E37 + F36 + FZ6;
N16 = CX6 - E27*r7 + MY6*VP25 - MZ6*VP26 + N17 + No16;
N26 = CY6 + E17*r7 - MX6*VP25 + MZ6*VP16 + N27 + No26;
N36 = CZ6 + MX6*VP26 - MY6*VP16 + N37 + No36;
FDI16 = C6*E16 - E26*S6;
FDI36 = -C6*E26 - E16*S6;
E15 = F15 + FDI16;
E25 = E36 + F25;
E35 = F35 + FDI36;
N15 = C6*N16 - MY5*VSP25 - MZ5*VP25 - N26*S6 + No15;
N25 = MX5*VSP25 + MZ5*VP15 + N36 + No25;
N35 = -C6*N26 + MX5*VP25 - MY5*VP15 - N16*S6 + No35;

FDI15 = C5*E15 - E25*S5;
FDI35 = C5*E25 + E15*S5;
E14 = F14 + FDI15;
E24 = -E35 + F24;
E34 = F34 + FDI35;
N14 = C5*N15 - FDI35*r5 + MY4*VSP34 - MZ4*VP24 - N25*S5 + No14;
N24 = -MX4*VSP34 + MZ4*VP14 - N35 + No24;
N34 = C5*N25 + FDI15*r5 + MX4*VP24 - MY4*VP14 + N15*S5 + No34;
FDI14 = C4*E14 - E24*S4;
FDI24 = C4*E24 + E14*S4;
E13 = F13 + FDI14;
E23 = F23 + FDI24;
E33 = E34 + F33;
N13 = C4*N14 - FDI24*r4 + MY3*VSP33 - MZ3*VP23 - N24*S4 + No13;
N23 = C4*N24 + E34*d4 + FDI14*r4 - MX3*VSP33 + MZ3*VP13 + N14*S4 + No23;
N33 = -FDI24*d4 + MX3*VP23 - MY3*VP13 + N34 + No33;
FDI13 = C3*E13 - E23*S3;
FDI23 = C3*E23 + E13*S3;
E12 = F12 + FDI13;
E22 = F22 + FDI23;
E32 = E33 + F32;
N12 = C3*N13 - MZ2*VP22 - N23*S3 + No12;
N22 = C3*N23 + E33*d3 + MZ2*VP12 + N13*S3 + No22;
N32 = -FDI23*d3 + MX2*VP22 - MY2*VP12 + N33 + No32;
FDI12 = C2*E12 - E22*S2;
FDI32 = C2*E22 + E12*S2;
E11 = F11 + FDI12;
E21 = -E32 + F21;
E31 = F31 + FDI32;
N11 = C2*N12 - G3*MY1 - N22*S2 + No11;
N21 = G3*MX1 - N32 + No21;
N31 = C2*N22 + N12*S2 + No31;
FDI11 = C1*E11 - E21*S1;
FDI21 = C1*E21 + E11*S1;
GAM1 = FS1*sign(QP1) + FV1*QP1 + IA1*QDP1 + N31;
GAM2 = FS2*sign(QP2) + FV2*QP2 + IA2*QDP2 + N32;
GAM3 = FS3*sign(QP3) + FV3*QP3 + IA3*QDP3 + N33;
GAM4 = FS4*sign(QP4) + FV4*QP4 + IA4*QDP4 + N34;
GAM5 = FS5*sign(QP5) + FV5*QP5 + IA5*QDP5 + N35;
GAM6 = FS6*sign(QP6) + FV6*QP6 + IA6*QDP6 + N36;
GAM7 = 0;
GAM=[GAM1;GAM2;GAM3;GAM4;GAM5;GAM6];
