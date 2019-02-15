function [ output_args ] = plotObjects( ObjectPositions,dimension )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

hold on

for i=1:size(ObjectPositions,1)
    if(dimension==3)
    plot3(ObjectPositions(i,1),...
        ObjectPositions(i,2),...
        2.0,...
        's',...
        'MarkerSize',15,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[1.0,1.0,1.0])
    else
        plot(ObjectPositions(i,1),...
        ObjectPositions(i,2),...
        's',...
        'MarkerSize',15,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[1.0,1.0,1.0])    
    end
end
end

