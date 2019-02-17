function [ V ] = getConstrainedManipulability(r2,q,qnames,objects,RobotChain,gains)
% getConstrainedManipulability get the constrained cartesian manipulability
% polytope, given the objects, joint position limits and joint velocity
% limits

[phi_max,phi_min]=getJointLimitPenalty(q(getIndices(qnames, RobotChain.LEFT_ARM )),RobotChain.LEFT_ARM_MAX_POSITION_LIMITS,RobotChain.LEFT_ARM_MIN_POSITION_LIMITS,4);
 PHI=eye(7*2)-diag([phi_max,phi_min]);
for j=1:length(RobotChain.LEFT_ARM)    
    T{j}=getTransform(r2,q,RobotChain.LEFT_ARM_LINKS{j});
    Jj=geometricJacobian(r2,q,RobotChain.LEFT_ARM_LINKS{j}); %    
    ind=getIndices(qnames, RobotChain.LEFT_ARM );
    J{j}=[Jj(4:6,ind);Jj(1:3,ind)];
end
T{length(RobotChain.LEFT_ARM) +1}=T{length(RobotChain.LEFT_ARM) }*RobotChain.Tne;
Lhat=skew(T{length(RobotChain.LEFT_ARM) }(1:3,1:3)*RobotChain.Tne(1:3,4));
eSn=[eye(3) -Lhat; zeros(3) eye(3) ];
J{length(RobotChain.LEFT_ARM) +1}=eSn*J{length(RobotChain.LEFT_ARM)};
%

dfdesired=gains.DANGER_VALUE;
A=[eye(7);-eye(7)];
B=[RobotChain.LEFT_ARM_MAX_VELOCITY_LIMITS;-RobotChain.LEFT_ARM_MIN_VELOCITY_LIMITS];
numberSteps=5;
for o=1:length(objects)
    for i=1:length(T)-1
        iTip1=inv(T{i})*T{i+1};
        iPip1=iTip1(1:3,4);
    if(norm(iPip1)<0.001)% Links are coicident
       %disp ' Links coincident'
    else
        for j=1:numberSteps
            distance_along_link=j*(T{i}(1:3,1:3)*iPip1)/numberSteps;
            cp=T{i}(1:3,4)+distance_along_link;
            Lhat=skew(distance_along_link);
            screwTransform=[eye(3) -Lhat; zeros(3) eye(3) ];
            JJ=screwTransform*J{i};
            obj_pt=objects{o}.project(cp);
            %  plotPoint(cp,'ko')
            %  plotPoint(obj_pt.x,'yo')            
             rdiff=obj_pt.x-cp;
            n=(rdiff)/norm(rdiff);
            
            if(norm(rdiff)<0.001)
                rdiff=0.001;
            elseif(norm(rdiff)<0.5) % 0.5 or 0.3
                A=[A;n'*JJ(1:3,:)];
                B=[B;(dfdesired*norm(rdiff)^2)-norm(rdiff)];
            end
            
        end        
    end
    end
end

Q=Polyhedron(A,[PHI*B(1:14);B(15:end)]);
try
    V=getCartesianPolytope(J{length(RobotChain.LEFT_ARM) +1},Q);
catch
  V=Polyhedron.emptySet(1);
end

end

