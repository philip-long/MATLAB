function N = checkEtaValue(n,Jc, V)
%checkEtaValue Calculates the eta value for a given configuration
% Eta is a value associated with a subchain of a complex 
% mechanism


N=zeros(size(V,1),1);
for i=1:size(V,1)
    v=Jc(1:3,:)*V(i,:)';
    N(i)=((n'*v));
end

end

