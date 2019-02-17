function CM = capacityMargin(Normals_hyerplanes, Dist_hyperplanes, ZMP)

Dist=[]; % store distances between each vertex of ZMP and each facet of the footprint

%%  For cables hyperplanes

for r=1:size(Normals_hyerplanes,2) % run for each facet of foot print
    al_1=Normals_hyerplanes(:,r);
    bl_1=norm(Dist_hyperplanes(1,r));
    
    for j=1:size(ZMP,2) % run for each vertex of ZMP
        Dist(j,r)=(bl_1-ZMP(:,j)'*al_1)/norm(al_1); % positive if inside
    end
end



%% Computation of the clossness Step (V)

CM=min(Dist); % All must be positive to ensure ZMP is inside of footprint


end