function [T0E,Jaco]=getEEInfo(T,J,TnE,frame,plotting)

T0E=T*TnE;
Q=rot2Quat_xyzw(T0E(1:3,1:3));
P=TnE(1:3,4);
L=T(1:3,1:3)*P;
Lhat=skew(L);
Screwtransform=[eye(3) -Lhat; zeros(3) eye(3) ];
Jaco=Screwtransform*J;

if(nargin<4)
    frame='torso'; 
end

if(nargin<5) % By default we plot, not a good solution.
    tftree = rostf;
    tform = rosmessage('geometry_msgs/TransformStamped');
    new_frame=strcat('control_point');
    tform.ChildFrameId = new_frame;
    tform.Header.FrameId = frame;
    tform.Transform.Translation.X = T0E(1,4);
    tform.Transform.Translation.Y = T0E(2,4);
    tform.Transform.Translation.Z = T0E(3,4);
    tform.Transform.Rotation.X=Q(1);
    tform.Transform.Rotation.Y=Q(2);
    tform.Transform.Rotation.Z=Q(3);
    tform.Transform.Rotation.W=Q(4);
    
    for j=1:10
        tform.Header.Stamp=rostime('now');
        sendTransform(tftree,tform)
        pause(0.05)
    end
end
end
