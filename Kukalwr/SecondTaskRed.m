function Z=SecondTaskRed(u)
% Copyright (c) 2012 Philip Long
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.
global X4d qinit

Kp=100;
Kd=500;
q=u(1:length(qinit));
qdot=u(length(qinit)+1:length(qinit)+7);

q=q(1:4);
qdot=qdot(1:4);



%disp ' X4d ='

X4=T40(q);
%disp ' X4 ='
X4=X4(1:3,4);

JE=J40(q);
X4dot=(JE)*qdot;
X4dot=X4dot(1:3);
JE=[JE(1:3,:) zeros(3,3)];
eddot=Kp*(X4d-X4)+Kd*(-X4dot);

Jpqp=JpQp40(u);
Jpqp=Jpqp(1:3);
Z=pinv(JE)*(eddot-Jpqp);