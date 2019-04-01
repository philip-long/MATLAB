function Pim=project2ImageSpace(X)

% this function projects a 3D in the camera frame to a point in image space using a pinhole camera mode
% 
% input X is either an array of 3D points or 1 3D point
%


% Collineation Matrix

%global CameraParameters
CameraParameters=[560; 560; 0;  374.4020879	;  180.2622981 ];
fu=CameraParameters(1);fv=CameraParameters(2);fs=CameraParameters(3);u0=CameraParameters(4);v0=CameraParameters(5);
K=[fu fs u0;0 fv v0; 0 0 1];


if size(X,1)==3
    Px=X(1,:);
    Py=X(2,:);
    Pz=X(3,:);
else 
    Xn=X';
    Px=Xn(1,:);
    Py=Xn(2,:);
    Pz=Xn(3,:);
end
% Px is a scalar or an vector of all the X points

% Normalise the image points

xn1=Px./Pz;yn1=Py./Pz;


% Convert Image Points
Pim=K*[xn1;yn1;ones(1,length(xn1))];
