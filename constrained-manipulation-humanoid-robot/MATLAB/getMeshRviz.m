function [ marker ] = getMeshRviz(t,frame,rgb_color)
%getMeshRviz For a set of vertices return a rviz object mesh message
%   Detailed explanation goes here
marker=rosmessage('visualization_msgs/Marker');
marker.Ns='Object';
marker.Action=marker.ADD;
marker.Type=marker.TRIANGLELIST;
marker.Id=randi(500);
marker.Header.FrameId=frame;


k=convhull(t.V);

% This is what you've reduced me to.
msgArrayg= [rosmessage('geometry_msgs/Point') ];
for i=1:(size(k,1)*3)-1
    msgArrayg=[msgArrayg rosmessage('geometry_msgs/Point') ];
end


ind=1;
for i=1:(size(k,1))
    for j=1:3
        point=k(i,j);
        msgArrayg(ind).X=t.V(point,1);
        msgArrayg(ind).Y=t.V(point,2);
        msgArrayg(ind).Z=t.V(point,3);
      ind=ind+1; 
    end    
end

marker.Points=msgArrayg;
marker.Lifetime = rosduration();

marker.Color.R=rgb_color(1);
marker.Color.G=rgb_color(2);
marker.Color.B=rgb_color(3);
marker.Color.A=1.0;
if(length(rgb_color)==4)
marker.Color.A=rgb_color(4);
end
marker.Scale.X=1.0;
marker.Scale.Y=1.0;
marker.Scale.Z=1.0;



end

