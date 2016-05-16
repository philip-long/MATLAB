


out=squeeze(q)';
t=out(:,1);
Q=out(:,2:end);
%%

for i=1:length(t)
    u=Q(i,:);
    global TnE
    
    T1=T10(u);
    T2=T20(u);
    T3=T30(u);
    
    TE=T3*TnE; %Tool Frame
    
    %% Locate Points
    P1=[0,0];
    P2=[T2(1,4),T2(2,4)];
    P3=[T3(1,4),T3(2,4)];
    PE=[TE(1,4),TE(2,4)];
    L1=[P1;P2;P3;PE];
    %% Plot Point
    
    plot(P1(1),P1(2),'o','MarkerSize',5,'MarkerFaceColor','g')
    axis([-1.5 1.5 -1.5 1.5])
    hold on
    plot(P2(1),P2(2),'o','MarkerSize',5,'MarkerFaceColor','g')
    plot(P3(1),P3(2),'o','MarkerSize',5,'MarkerFaceColor','g')
    plot(PE(1),PE(2),'h','MarkerSize',20,'MarkerFaceColor','R')
    plot(L1(:,1),L1(:,2),'k','LineWidth',2)
    hold off
    pause(0.1)
end
pause(0.5)
close all