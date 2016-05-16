% This curve is the key really showing the force
% comparison

close all,clear all,clc
LastDataPoint=[1,382,819,1304,1833,2379,2954,3562,4206,4865,5551,6259,7008,7756,8544];
for i=(1:length(LastDataPoint)-1)
    Prnge{i}=LastDataPoint(i):LastDataPoint(i+1);
end
F=load('MovingAverageForce');
V=load('OutputCmdtV');
data = load('bMt');
Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);


for i=1:14
R=Prnge{i};
figure(1)
plot(F(R,2),'r*')
figure(2)

pause()
end
%%
for i=1:14
    plot(V(R,3),'b:')
    hold on
    plot(Pz(R))
end

for i=1:14
    R=Prnge{i};
    plot(Px(R),Pz(R),'*')
    
    hold on
    plot(Px(R),F(R,2),'r*') 
    pause()
end
%%
subplot(2,3,1)
R=Prnge{1};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 1')

subplot(2,3,2)
R=Prnge{3};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 3')



subplot(2,3,3)
R=Prnge{5};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 5')


subplot(2,3,4)
R=Prnge{7};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 7')


subplot(2,3,5)
R=Prnge{9};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 9')


subplot(2,3,6)
R=Prnge{11};
[AX,H1,H2]=plotyy(Px(R),Pz(R),Px(R),-F(R,2),'plot')
set(get(AX(1),'Ylabel'),'String',' Pz(m)') 
set(get(AX(1),'Xlabel'),'String','Px(m)') 
set(AX,'xlim',[-.655,-.54]);
set(AX(1),'ylim',[0.1,0.15]);set(AX(2),'ylim',[0.0,4]);
set(AX(1),'ytick',[0.1,0.125,0.15]);set(AX(2),'ytick',[1,2,3,4]);
set(get(AX(2),'Ylabel'),'String','Cutting Force')
set(H1,'linestyle','none','marker','.'); 
set(H2,'linestyle','none','marker','s');
title('Passage 11')
legend('Pz','Cutting Force')
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0
1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Cutting Force V Displacement','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',20)

%%
R=Prnge{8};
Fres=(F(R,2).^2+(F(R,3).^2)).^0.5
plot(Px(R),Fres,'g*')
hold on
plot(Px(R),F(R,2),'r*')
plot(Px(R),F(R,3),'b*')
