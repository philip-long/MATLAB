function Soln_Set=Kuka_6ax_simple_pieper(Td)
global alpha
%% Define the parameters
Px = Td(1,4);
Py = Td(2,4);
Pz = Td(3,4);
sx = Td(1,1);
sy = Td(2,1);
sz = Td(3,1);
nx =Td(1,2);
ny = Td(2,2);
nz = Td(3,2);
ax = Td(1,3);
ay = Td(2,3);
az = Td(3,3);
te=alpha(3);




%% t3: 2 solution depending and e
Counter=0;
for e3=-1:2:1 % Finding the different solutions via for loops
    for e2=-1:2:1
        for e4=-1:2:1
            Counter=Counter+1;
            TCy=0.39*cos(te);
            TCz=0.39*sin(te);
            X=0;
            Z=-0.16000000000000003 + Px^2 + Py^2 + Pz^2 - TCy^2 - TCz^2;
            SQ = (-0.31200000000000006*Z + e3*X*(0.09734400000000003 + X^2 - Z^2)^0.5)/(0.09734400000000003 + X^2);
            CQ = (X*Z + 0.31200000000000006*e3*(0.09734400000000003 + X^2 - Z^2)^0.5)/(0.09734400000000003 + X^2);
            t3 = atan2( SQ , CQ );
            
            %t2: 2 solutions depending on e
            
            Fx=0.4 - 0.39*sin(t3);
            Fy=TCy*cos(t3);
            Fz=TCz*cos(t3);
            SQ = (Fx*Pz + e2*Fy*(Fx^2 + Fy^2 - Pz^2)^0.5)/(Fx^2 + Fy^2);
            CQ = (Fy*Pz - e2*Fx*(Fx^2 + Fy^2 - Pz^2)^0.5)/(Fx^2 + Fy^2);
            t2 = atan2( SQ , CQ );
            
            
            %t1:
            c=-(Fx*cos(t2)) + Fy*sin(t2);
            X=Px^2 + Py^2;
            Y=Fz*Px - c*Py;
            X1=-Px^2 - Py^2;
            Y1=c*Px + Fz*Py;
            
            t1 = atan2(Y/X , Y1/X1 );
            
            
            
            %%%%%%%%%%%%%%%%%%%%%
            U1T21=-(cos(te)*sin(t2));
            U1T22=cos(t2)*cos(te);
            U1T31=sin(t2)*sin(te);
            U1T32=-(cos(t2)*sin(te));
            U1T111=cos(t2)*cos(t3) + U1T21*sin(t3);
            U1T112=cos(t3)*sin(t2) + U1T22*sin(t3);
            U1T113=sin(t3)*sin(te);
            U1T114=-0.4*cos(t3);
            U1T121=U1T21*cos(t3) - cos(t2)*sin(t3);
            U1T122=U1T22*cos(t3) - sin(t2)*sin(t3);
            U1T123=cos(t3)*sin(te);
            U1T124=0.4*sin(t3);
            U1T311=U1T111*cos(t1) + U1T113*sin(t1);
            U1T312=-(U1T113*cos(t1)) + U1T111*sin(t1);
            U1T321=U1T121*cos(t1) + U1T123*sin(t1);
            U1T322=-(U1T123*cos(t1)) + U1T121*sin(t1);
            U1T331=U1T31*cos(t1) + cos(te)*sin(t1);
            U1T332=-(cos(t1)*cos(te)) + U1T31*sin(t1);
            SNA11=sz*U1T112 + sx*U1T311 + sy*U1T312;
            SNA12=nz*U1T112 + nx*U1T311 + ny*U1T312;
            SNA13=az*U1T112 + ax*U1T311 + ay*U1T312;
            SNA21=-(sz*U1T32) - sx*U1T331 - sy*U1T332;
            SNA22=-(nz*U1T32) - nx*U1T331 - ny*U1T332;
            SNA23=-(az*U1T32) - ax*U1T331 - ay*U1T332;
            SNA31=sz*U1T122 + sx*U1T321 + sy*U1T322;
            SNA32=nz*U1T122 + nx*U1T321 + ny*U1T322;
            SNA33=az*U1T122 + ax*U1T321 + ay*U1T322;
            
            
            %t4: 4
            if e4==1
                t4 = atan2(SNA23,SNA13);
            else
                t4 = atan2(-SNA23,-SNA13 );
            end
            
            %t5: 4 solution depending on t1 and t4
            B10=-(SNA13*cos(t4)) - SNA23*sin(t4);
            t5 = atan2(B10 , SNA33 );
            
            %t6: 4 solution depending on t1 and t4
            B10=SNA21*cos(t4) - SNA11*sin(t4);
            B20=SNA22*cos(t4) - SNA12*sin(t4);
            
            t6 = atan2(B10 ,B20);
            Soln_Set{Counter}=[t1 t2 t3 t4 t5 t6];
        end
    end
end
















