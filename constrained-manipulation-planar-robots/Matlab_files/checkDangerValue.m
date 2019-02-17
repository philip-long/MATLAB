function N = checkDangerValue(n,Jc, V,rdiff)
%checkDangerValue Calculates the danger value for a given configuration
N=zeros(size(V,1),1);
for i=1:size(V,1)
    v=Jc(1:3,:)*V(i,:)';
    N(i)=((n'*v)+norm(rdiff))/norm(rdiff)^2;
end

end

