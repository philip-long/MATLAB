function [ Pv ] = getCartesianPolytope(J,PQ)
%getCartesianPolytope Gets Cartesian Polytope for translational velocities
V_obs=zeros(length(PQ.V),2);
for i=1:length(PQ.V)
V_obs(i,:)=(J*( PQ.V(i,:))')';
end
Pv=Polyhedron(V_obs);

end

