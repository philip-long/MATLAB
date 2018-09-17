function [A,Lx,Ly,Px,Py,Fx,Fy]=GetMoments(M)

M = double(M);
TAM=size(M);
X=TAM(1);
Y=TAM(2);
BOX = X*Y;


%Area
A=sum(sum(M));
%[dX,dY]=gradient(M);

dY =  diff(M,1,1);
dX =  diff(M,1,2);

mDx=sum(sum(abs(dX)));
mDy=sum(sum(abs(dY)));

ly = 1:X-1;
lx = 1:Y-1;

% Longitud de Superficie Surface length
Lx=(1.0+sum(1.0+sum(sqrt(1.0+(dX.^2.0)),2))) / BOX ;
Ly=(1.0+sum(1.0+sum(sqrt(1.0+(dY.^2.0)),2))) / BOX ;

    
% Estacion   Position components
Px=( 1+sum(abs(dX)*lx') ) ./ (0.1+(mDx/(4.5)));
Py=( 1+sum(ly*abs(dY) ) ) ./ (0.1+(mDy/(8.0)));

Fx=1;
Fy=1;



end


