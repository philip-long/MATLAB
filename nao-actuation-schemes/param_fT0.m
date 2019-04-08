function [gamma, b, alpha, d, theta, r]= param_fT0(T)
% Ce programme permet de calculer les 6 param�tres
% d�finissant le rep�re zero dans le rep�re fixe (R-1), en utilisant la
% matrice de transformation homog�ne fT0, avec f le rep�re fixe de la cam�ra et 0 le rep�re 0 du robot
% puis on calcule les six param�tres (gamma,b, alpha, d, theta, r) selon l'algorithme suivant:
%
% - les angles g, a, t s'obtiennent par les formules des angles d'Euler
%   suivant la combinaison rot(z,g)rot(x,a)rot(z,t)(livre khalil et Dombre 99, page 72-73)
%     - g = atan2(-ax,ay) ou g = atan2(ax,-ay)
%     - a = atan2(sin(g)*ax - cos(g)*ay, az)
%     - t = atan2(-cos(g)*nx - sin(g)*ny, cos(g)*sx+sin(g)*sy)
% - A partir de l'�quation (7.3) on obtient:
%     - Px= cos(g)*d+sin(g)*sin(a)*r, Py=sin(g)*d-cos(g)*sin(a)*r
%     - donc d= Px*cos(g)+ Py*sin(g)
%     - donc Px*sin(g)-py*cos(g)=r*sin (a)
%     - donc r= (Px*sin(g)-py*cos(g))/sin(a)
%     - Finalement b= Pz-r*cos(a).
%\
%    Exemple : soit la matrice de transformation cT0 = [s n a P1], on calcule les 6 param�tres de cette derni�re transformation
% selon l'algorithme ci-dessus.

% donn�es
sx=T(1,1);
sy=T(2,1);
sz=T(3,1);
nx=T(1,2);
ny=T(2,2);
nz=T(3,2);
ax=T(1,3);
ay=T(2,3);
az=T(3,3);
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);
% premi�re solution : g= atan2(-ax,ay)
eps=0.00001;
if (abs(ax)<eps && abs(ay)<eps)
    alpha=0;
    gamaplusthe=atan2(-nx,ny);% pas possible d'avoir aussi nx=ny=0
    if (abs(Px)<eps && abs(Py)<eps)
        % d=0;
        gamma=0; % gamma quelconque
    else
        gamma=atan2(Py,Px);
    end
    theta=gamaplusthe-gamma;
else
    gamma= atan2(ax,-ay);
    alpha = atan2(sin(gamma)*ax - cos(gamma)*ay, az);
    theta = atan2(-cos(gamma)*nx - sin(gamma)*ny, cos(gamma)*sx+sin(gamma)*sy);
end;
d = Px*cos(gamma)+ Py*sin(gamma);
if (abs(sin(alpha))<eps)
    b=0;
    r=Pz;
else
    r=(Px*sin(gamma)-Py*cos(gamma))/sin(alpha);
    b= Pz-r*cos(alpha);
end;

%disp ('   premi�re solution gamma, b, alpha, d, theta, r');


% v�rification premi�re solution par le calcul de la matrice directe Eq.7.3
g=gamma;
t=theta;
a=alpha;
Tva(1,1)= cos(g)*cos(t)-sin(g)*cos(a)*sin(t);
Tva(2,1)= sin(g)*cos(t)+ cos(g)*cos(a)*sin(t);
Tva(3,1)= sin(a)*sin(t);
Tva(1,2)= -cos(g)*sin(t)-sin(g)*cos(a)*cos(t);
Tva(2,2)= -sin(g)*sin(t)+ cos(g)*cos(a)*cos(t);
Tva(3,2)= sin(a)*cos(t);
Tva(1,3)= sin(g)*sin(a);
Tva(2,3)= -cos(g)*sin(a);
Tva(3,3)= cos(a);
Tva(1,4)= d*cos(g)+r*sin(g)*sin(a);
Tva(2,4)= d*sin(g)- r* cos(g)*sin(a);
Tva(3,4)= r*cos(a)+b;
Tva(4,1)= 0;
Tva(4,2)= 0;
Tva(4,3)= 0;
Tva(4,4)= 1;
erreur1 = Tva-T;
% deuxi�me solution
%g= atan2(-ax,ay)

if (abs(ax)<eps && abs(ay)<eps)
    alpha=0;
    gamaplusthe=atan2(-nx,ny);% pas possible d'avoir aussi nx=ny=0
    if (abs(Px)<eps && abs(Py)<eps)  % d=0;
        gamma=0; % gamma quelconque
    else
        gamma=atan2(Py,Px);
    end
    theta=gamaplusthe-gamma;
else
    gamma= atan2(-ax,ay);
    alpha = atan2(sin(gamma)*ax - cos(gamma)*ay, az);
    theta = atan2(-cos(gamma)*nx - sin(gamma)*ny, cos(gamma)*sx+sin(gamma)*sy);
end;
d = Px*cos(gamma)+ Py*sin(gamma);
if (abs(sin(alpha))<eps)
    b=0;
    r=Pz;
else
    r=(Px*sin(gamma)-Py*cos(gamma))/sin(alpha);
    b= Pz-r*cos(alpha);
end;

%disp ('  deuxi�me solution gamma, b, alpha, d, theta, r');



% v�rification deuxi�me solution Eq.7.3
g=gamma;
t=theta;
a=alpha;
Tva(1,1)= cos(g)*cos(t)-sin(g)*cos(a)*sin(t);
Tva(2,1)= sin(g)*cos(t)+ cos(g)*cos(a)*sin(t);
Tva(3,1)= sin(a)*sin(t);
Tva(1,2)= -cos(g)*sin(t)-sin(g)*cos(a)*cos(t);
Tva(2,2)= -sin(g)*sin(t)+ cos(g)*cos(a)*cos(t);
Tva(3,2)= sin(a)*cos(t);
Tva(1,3)= sin(g)*sin(a);
Tva(2,3)= -cos(g)*sin(a);
Tva(3,3)= cos(a);
Tva(1,4)= d*cos(g)+r*sin(g)*sin(a);
Tva(2,4)= d*sin(g)- r* cos(g)*sin(a);
Tva(3,4)= r*cos(a)+b;
Tva(4,1)= 0;
Tva(4,2)= 0;
Tva(4,3)= 0;
Tva(4,4)= 1;
erreur2= Tva-T;
