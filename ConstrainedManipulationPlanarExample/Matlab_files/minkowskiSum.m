function [ C ] = minkowskiSum( A,B )
%MinkowskiSum MinkowskiSum of two polytopes
%   Pairwise summation of two convex polytopes
%   assuming both are defined in the same dimensional space.

if(~(size(A,2)==size(B,2)))
    error('Polytope dimension incongurity')
end

C=(zeros(size(A,1)*size(B,1),size(A,2)));
c=0;
for i=1:size(A,1)
    for j=1:size(B,1)
    c=c+1;
    C(c,:)=A(i,:)+B(j,:);
    end
end


end

