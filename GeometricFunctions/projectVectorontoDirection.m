function [v1_proj_mag,v1_proj]=projectVectorontoDirection(v1,v2,plotting)
% projectVector projection of v1 onto v2
% 
% returns v1_proj_mag the maginutude of the projection of v1 on v2
%         v1_proj the vector of the resulting projection (will be in the
%         same direction as v2)

if(size(v1,1)<size(v1,2))     v1=v1'; end
if(size(v2,1)<size(v2,2))     v2=v2'; end

v2hat=v2/norm(v2);
v1_proj_mag=(v1'*v2hat); % Check if positive
v1_proj=v1_proj_mag*v2hat; 


% plotting

if(nargin==3)
    if(plotting)
        if(length(v2)==2)
            plot([0;-v1(1)],[0;-v1(2)],'b');
            hold on
            plot([0;-v2(1)],[0;-v2(2)],'r');
            plot([0;-v1_proj(1)],[0;-v1_proj(2)],'y:','LineWidth',2.0);
            legend('a','b','a onto b')
        elseif(length(v2)==3)
            
            plot3([0;-v1(1)],[0;-v1(2)],[0;-v1(3)],'b');
            hold on
            plot3([0;-v2(1)],[0;-v2(2)],[0;-v2(3)],'r');
            plot3([0;-v1_proj(1)],[0;-v1_proj(2)],[0;-v1_proj(3)],'y:','LineWidth',2.0);
            legend('a','b','a onto b')
        end
    end
    axis('equal')
end


