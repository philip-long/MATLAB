function Q= expandPolytope(polytopem1,qu,ql)
%expandPolytope Expands polytope to the nex dimension given new dimension
%upper and lower limits
%   Detailed explanation goes here

%     if(polytopem1==0)
%         Q=[qu;ql];
%     else
    QU=ones(max(length(polytopem1),1),1)*qu;
    QL=ones(max(length(polytopem1),1),1)*ql;
    Q=[ [polytopem1;polytopem1],[QU;QL]];
%     end
end
