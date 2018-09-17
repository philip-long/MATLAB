%% Test to see how redundant are redundant manipulators
%
clear all, clc



global r1 r3 r5 r7 d3 d4
r1=0.5;r3=0.39;r5=0.4;r7=0.6;
d3=0.5; d4=-0.5;
% This code deletes the effect of one joint to see
% the rank of the new Jacobian matrix for a random configuration
% I thought that if one joint is frozen the mechanism still has
% 6 DOF but occasionally by freezing certain joints the Jacobian
% loses both its "redundancy" and also a "DOF"

u=rand(7,1)*2*pi-(pi) % u between pi and -pi
%% Case 1 Kuka lwr
J=J70(u)

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    i;
    rank(Jr);
    if rank(Jr)<6
        
        disp 'Kuka lwr:Degenracy caused by fixing joint '
        i
        pause()
    end
end


% Result:
%Lose rank when 4 is fixed

%% Case 2 Mitubishi PA 10
J=Pa10(u)

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    rank(Jr)
    if rank(Jr)<6
        disp 'Mitubishi PA 10: Degenracy caused by fixing joint'
        i
        pause()
    end
end
% Result:
%Lose rank when 1,2,3,4,5 is fixed


%% Case 3 external axis is prismatic
% NO Degeneracy

J=KukawithPrism(u);

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    rank(Jr)
    if rank(Jr)<6
        disp 'KukawithPrism: Degenracy caused by fixing joint'
        i
        pause()
    end
end


%%

%% Case 4 Prismatic axis placed at tool tip
% NO Degeneracy

J=KukawithPrism2(u)

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    rank(Jr)
    if rank(Jr)<6
        disp 'KukawithPrism2:Degenracy caused by fixing joint'
        i
        pause()
    end
end


%% Case 5 Offset added to external axis
% No degeneracy

J=ExOffset(u)

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    rank(Jr) ;
    if rank(Jr)<6
        pause()
        disp 'ExOffset:Degenracy caused by fixing joint'
        i
    end
    
end

%% Case 6 Offset added to external axis
% No degeneracy

J=ExOffsetandCancel(u)

for i=1:7 % r for reduced
    Jr=J;
    Jr(:,i)=[];
    
    if rank(Jr)<6
        pause()
        disp 'ExOffsetandCancel:Degenracy caused by fixing joint',
        i
    end
end







