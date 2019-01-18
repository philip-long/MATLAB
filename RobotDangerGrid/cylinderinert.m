function I=cylinderinert(radius,length,mass)


ixx=0.0833333 * mass * (3 * radius * radius + length * length);
ixy=0.0;
ixz=0.0;
iyy=0.0833333 * mass * (3 * radius * radius + length * length);
iyz=0.0;
izz=0.5 * mass * radius * radius;

I=[ixx -ixy -ixz
   -ixy iyy -iyz
	-ixz iyz izz];
end
