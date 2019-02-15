function [ output_args ] = plotPlanarRobot(T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



plot3([0.0,T{1}(1,4)],[0.0,T{1}(2,4)],[1.2,1.2],'ks','LineWidth',8.0);
hold on
plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[2.2,2.2],'-s','color',[0.3,0.3,0.3],...
    'LineWidth',5.0,'MarkerSize',2.0,'MarkerFaceColor', [0.0 0.0 0.0],'MarkerEdgeColor', [0.0 0.0 0.0]);
plot3([T{2}(1,4),T{3}(1,4)],[T{2}(2,4),T{3}(2,4)],[2.2,2.2],'-s','color',[0.3,0.3,0.3],...
    'LineWidth',5.0,'MarkerSize',2.0,'MarkerFaceColor', [0.0 0.0 0.0],'MarkerEdgeColor', [0.0, 0.0 0.0]);
%plot3([T{1}(1,4),T{2}(1,4)],[T{1}(2,4),T{2}(2,4)],[1.3,1.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);
%plot3([T{2}(1,4),T{3}(1,4)],[T{2}(2,4),T{3}(2,4)],[1.3,1.3], 'color', [0.5 0.5 0.5],'LineWidth',1.0);



end

