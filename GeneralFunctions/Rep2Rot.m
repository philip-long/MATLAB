function R=Rep2Rot(u)

c=u(end);


switch c
    case 1
        Q=u(1:4);
        R=quaternion2matrix(Q);
    case 2 %Euler Angles
        phi=u(1);
        theta=u(2);
        if theta==0;
            pause()
        end
        psi=u(3);
        R=TransMat(phi,'z','rot')*TransMat(theta,'x','rot')*TransMat(psi,'z','rot')
           
       
    case 4
        phi1=u(1);
        theta1=u(2);
        if theta1==pi/2;
            pause()
        end
        psi1=u(3);
        
       
        R=TransMat(phi1,'z','rot')*TransMat(theta1,'y','rot')*TransMat(psi1,'x','rot')
        
end