function [N,Nnot]=getDofCombinations(active_joints,m)

% We select m-1 independent twists that define m contained in N
%    the rest n-(m-1) are contained in N-1
% This is repeated for all combinations and pairs are corresponding rows in
% N and Nnot
    N=combnk(active_joints,m-1);
    Nnot=zeros(size(N,1),length(active_joints)-(m-1));
    for i=1:size(N,1)
        k=1;
        for j = 1:length(active_joints)
            if(~any(N(i,:)==j))
                Nnot(i,k)=j;
                k=k+1;
            end
        end
    end
    
end