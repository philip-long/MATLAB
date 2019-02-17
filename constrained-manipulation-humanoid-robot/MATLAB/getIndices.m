function [ idx ] = getIndices(VectorNames, Names )
%getIndices Get the indices corresponding to a vector of strings in a
%larger  vector
% 


idx=zeros(7,1);
for i=1:length(Names)
    idx(i) = find(strcmp(VectorNames, Names(i)));
end

end

