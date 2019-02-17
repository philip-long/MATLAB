function [ Pv ] = getCartesianPolytope(J,PQ)
%getCartesianPolytope Transform configuration space Polytope to 
%   Cartesian space

V_obs=zeros(length(PQ.V),3);
for i=1:length(PQ.V)
V_obs(i,:)=(J(1:3,:)*( PQ.V(i,:))')';
end
Pv=Polyhedron(V_obs);

end
