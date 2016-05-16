function T= DGM_leg_sym( q,Leg,Frame)
%DGM_leg DGM of Tree strcutured robot
%   This function returns the frame $Frame$ of the leg
%   $Leg$ of the Gough stewart tree structured
%   equivalent robot. This function is not
%   optimized rather it is used for debugging and
%   offline purposes

Leg1Origin=[-0.25,-0.433];
Tw0=[1,0,0,-0.25;    0,1,0,-0.433;    0,0,1,0;    0,0,0,1];
n=3;
switch (Leg)
    case 1
        
        alpha=sym([-pi/2,pi/2,pi/2]); d=[0,0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[0.0,0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
  
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
    case 2
        %Leg 2
        Leg2Origin=[0.75,-0.433];
        alpha=sym([-pi/2,pi/2,pi/2]); d=[dist2Dpts(Leg1Origin,Leg2Origin),0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[0.0,0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
        
        %Leg 3
    case 3
        Leg3Origin=[1.25,0.433];
        alpha=sym([-pi/2,pi/2,pi/2]); d=[dist2Dpts(Leg1Origin,Leg3Origin),0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[pi/6,0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
        
    case 4
        %Leg 4 ok
        Leg4Origin=[0.75,1.3];
        n=3;
        alpha=sym([-pi/2,pi/2,pi/2]); d=[dist2Dpts(Leg1Origin,Leg4Origin),0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[pi/3,0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
        
        %Leg 5 OK
    case 5
        Leg5Origin=[-0.25,1.3];
        n=3;
        alpha=sym([-pi/2,pi/2,pi/2]); d=[dist2Dpts(Leg1Origin,Leg5Origin),0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[pi/2,0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
        
        
        %Leg 6 OK
    case 6
        Leg6Origin=[-0.75,0.433];
        n=3;
        alpha=sym([-pi/2,pi/2,pi/2]); d=[dist2Dpts(Leg1Origin,Leg6Origin),0,0]; theta=sym([0,0,0]); r=sym([0,0,0]);
        sigma=[0,0,1];gamma=[2*pi/3,0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=sym(zeros(1,length(r)));
        t=sym(zeros(1,length(theta)));
        
        for j=1:n
            if sigma(j)==1
                rr(j)=q(j)+r(j);
                t(j)=theta(j);
            else
                t(j)=q(j)+theta(j);
                rr(j)=r(j);
            end
            Tzb=[ 1, 0, 0, 0
                0, 1, 0, 0
                0, 0, 1, b(j)
                0, 0, 0, 1];
            Tzg=[ cos(gamma(j)), -sin(gamma(j)), 0, 0
                sin(gamma(j)),  cos(gamma(j)), 0, 0
                0,           0, 1, 0
                0,           0, 0, 1];
            
            Tj=Tzb*Tzg*[cos(t(j)) -sin(t(j)) 0 d(j)
                cos(alpha(j))*sin(t(j)) cos(alpha(j))*cos(t(j)) -sin(alpha(j)) -rr(j)*sin(alpha(j))
                sin(alpha(j))*sin(t(j)) sin(alpha(j))*cos(t(j)) cos(alpha(j)) rr(j)*cos(alpha(j))
                0 0 0 1];
            T0n=T0n*Tj;
            T{j}=T0n;
        end
        
        T=Tw0*T{Frame};
    otherwise
        disp 'outside limits'
end


end

