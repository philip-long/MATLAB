function plotPlanarRobot(T,dimension)
%plotPlanarRobot Matlab plot of planar robot
%   Detailed explanation goes here


for i=1:(length(T)-1)
     if(dimension==3)
plot3([T{i}(1,4),T{i+1}(1,4)],[T{i}(2,4),T{i+1}(2,4)],[1.2,1.2],...
    '-ks', 'LineWidth',2.0,...
    'MarkerSize',10.0,'MarkerEdgeColor', 'b','MarkerFaceColor', rand(1,3)*i/length(T));
hold on
     else
      plot([T{i}(1,4),T{i+1}(1,4)],[T{i}(2,4),T{i+1}(2,4)],...
    '-ks', 'LineWidth',2.0,...
    'MarkerSize',10.0,'MarkerEdgeColor', 'b','MarkerFaceColor', rand(1,3)*i/length(T));
hold on   
     end
end


end

