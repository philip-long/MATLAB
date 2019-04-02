function  drawframe2D( T,scale )
% Writing my own god dawn function because all the
% rest take external libraries, based on
% $Id: drawframe.m,v 1.1 2009-03-17 16:40:18 bradleyk Exp $
% Copyright (C) 2005, by Brad Kratochvil I
% apologise Brad it was a Matlab issue

  p = T(1:3,4);
  r = T(1:3,1:3);

plot(p(1),p(2),'ko') % plot origin of the frame
hold on
Xaxisr=scale*r(1:3,1);
Yaxisr=scale*r(1:3,2);
% plot Xaxis
plot([p(1),p(1)+Xaxisr(1)],[p(2),p(2)+Xaxisr(2)],'r');
plot([p(1),p(1)+Yaxisr(1)],[p(2),p(2)+Yaxisr(2)],'b');

