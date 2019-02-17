function [ Q ] = generateVertexSet(ub,lb)
%generateVertexSet Generate a V repsentation of a polytope based on upper
%and lower bounds


assert(length(lb)==length(ub),'Upper bounds and lower bounds must have the same length!');
Q=[];
    for i=1:length(ub)
    Q=expandPolytope(Q,lb(i),ub(i));
    end     
end

