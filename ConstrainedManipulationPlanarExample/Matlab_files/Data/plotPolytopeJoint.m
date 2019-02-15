function plotPolytopeLin(polytope_vertices,face_color,face_alpha,figure_handle,v)

r=rank(polytope_vertices);
fillit=false;
if(r~=0)
    switch nargin
        case 1
            face_color='red';
            line_style=strcat('r','-s');
            face_alpha=0.1;
            
        case 2
            line_style=strcat(face_color,'s');
            face_alpha=0.1;
        case 4
            line_style=strcat(face_color,'s');
            figure(figure_handle);
        case 5
            line_style=strcat(face_color,'-s');
            figure(figure_handle);
            fillit=v;
    end
    hold on
    
    
    switch(r)
        case 1
            disp '1 DOF plotting line'
            plot3(polytope_vertices(:,1),polytope_vertices(:,2),polytope_vertices(:,3),line_style,'LineWidth',2.0)
        case 2
            
            disp 'degenerates to surface, simulating 0 thickness, '
            if(size(polytope_vertices,2)==2)
                new_poly=polytope_vertices;
                [k vol]=convhull(new_poly(:,1),new_poly(:,2));
                plot(polytope_vertices(k,1),polytope_vertices(k,2),line_style,'LineWidth',2.0)  
                if(fillit)
                fill(polytope_vertices(k,1),polytope_vertices(k,2),[0.3 0.3 0.3],'facealpha',0.2)
                end                   
            elseif(all(polytope_vertices(:,3)==0))
                new_poly=polytope_vertices;
                [k vol]=convhull(new_poly(:,1),new_poly(:,2));
                vol
                plot3(polytope_vertices(k,1),polytope_vertices(k,2),zeros(length(k),1),line_style,'LineWidth',2.0)  
                if(fillit)
                fill(polytope_vertices(k,1),polytope_vertices(k,2),[0.3 0.3 0.3],'facealpha',0.2)
                end                
            else
            n=cross(polytope_vertices(1,:),polytope_vertices(2,:));
            R=vrrotvec2mat(vrrotvec(n,[0;0;1]));
            new_poly=zeros(size(polytope_vertices));
            for i=1:size(polytope_vertices,1)
                new_poly(i,:)=(R*polytope_vertices(i,:)' )';
            end
            k=convhull(new_poly(:,1),new_poly(:,2));
            
            plot3(polytope_vertices(k,1),polytope_vertices(k,2),polytope_vertices(k,3),line_style,'LineWidth',2.0)
            fill(polytope_vertices(k,1),polytope_vertices(k,2),[0.3 0.3 0.3],'facealpha',0.2);
            end
        case 3
            [K2 v]=convhull(polytope_vertices);
            title('3 Dimensions')
            trisurf(K2,polytope_vertices(:,1),polytope_vertices(:,2),polytope_vertices(:,3),'Facecolor',face_color,'FaceAlpha',face_alpha);
            
    end
    xlabel('$\dot{q}_{1} [s^{-1}]$','Interpreter','latex')
    ylabel('$\dot{q}_{2} [s^{-1}]$','Interpreter','latex')
    zlabel('v_{z} [m s^{-1}]','Interpreter','Tex')
    
else
    disp 'Empty Polytope'
end
end