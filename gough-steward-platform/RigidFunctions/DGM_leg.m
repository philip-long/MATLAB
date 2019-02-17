function T= DGM_leg( q,Leg,Frame)
%DGM_leg DGM of Tree strcutured robot
%   This function returns the frame $Frame$ of the leg
%   $Leg$ of the Gough stewart tree structured
%   equivalent robot. This function is not
%   optimized rather it is used for debugging and
%   offline purposes

global Leg1Origin Leg2Origin Leg3Origin Leg4Origin Leg5Origin Leg6Origin Tw0

n=3;
switch (Leg)
    case 1
        
        alpha=[-pi/2,pi/2,pi/2]; d=[0,0,0]; theta=sym([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[0.0,0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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
   
        alpha=[-pi/2,pi/2,pi/2]; d=[dist2Dpts(Leg1Origin,Leg2Origin),0,0]; theta=sym([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[0.0,0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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
   
        alpha=[-pi/2,pi/2,pi/2]; d=[dist2Dpts(Leg1Origin,Leg3Origin),0,0]; theta=([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[atan2(Leg3Origin(2)-Leg1Origin(2),Leg3Origin(1)-Leg1Origin(1)),0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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
        n=3;
        alpha=[-pi/2,pi/2,pi/2]; d=[dist2Dpts(Leg1Origin,Leg4Origin),0,0]; theta=([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[atan2(Leg4Origin(2)-Leg1Origin(2),Leg4Origin(1)-Leg1Origin(1)),0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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
      
        n=3;
        alpha=[-pi/2,pi/2,pi/2]; d=[dist2Dpts(Leg1Origin,Leg5Origin),0,0]; theta=([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[atan2(Leg5Origin(2)-Leg1Origin(2),Leg5Origin(1)-Leg1Origin(1)),0.0,0.0];b=[0.0,0.0,0.0];
        
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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
    
        n=3;
        alpha=[-pi/2,pi/2,pi/2]; d=[dist2Dpts(Leg1Origin,Leg6Origin),0,0]; theta=([0,0,0]); r=[0,0,0];
        sigma=[0,0,1];gamma=[atan2(Leg6Origin(2)-Leg1Origin(2),Leg6Origin(1)-Leg1Origin(1)),0.0,0.0];b=[0.0,0.0,0.0];
        T=cell([n 1]);
        T0n=eye(4);
        rr=zeros(1,length(r));
        t=zeros(1,length(theta));
        
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

