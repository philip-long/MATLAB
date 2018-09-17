% This program is to formulate a method of
% identifying wheather two line segments defined
% by [P1,P2] and [P3,P4] interesect or not. The
% idea is to find a simple function that can
% transfomed into a fotran code for use in an MSC
% ADams simulator




while(1)
% define the points    
EE=(rand(3,1)*2)-1;
% EE=[0.1;0.1;-0.1]
EE2=[EE(1);EE(2);EE(3)+2];
P3=[(rand(3,1)*2)-1];
P4=[(rand(3,1)*2)-1];

Det=det([P3(1) P4(1) EE(1)
    P3(2) P4(2) EE(2)
    1  1 1]);

EE(1)-P3(1)/EE(1)-P4(1)


if EE(3)<min(P3(3),P4(3)) && abs(Det)<0.001 && EE(1)>min(P3(1),P4(1)) && EE(2)>min(P3(2),P4(2)) && EE(1)<max(P3(1),P4(1)) && EE(2)<max(P3(2),P4(2))
     disp 'INTERSECTING with determinant method'
     plot3([EE(1); EE2(1)],[EE(2);EE2(2)],[EE(3);EE2(3)])
    hold on
    pause(0.1)
    plot3([P3(1);P4(1)],[P3(2);P4(2)],[P3(3);P4(3)],'r')
    
    pause()
end
 hold off 

if EE(3)<min(P3(3),P4(3)) && abs(((P4(2)-P3(2))*(P4(1)-EE(1)))-((P4(1)-P3(1))*(P4(2)-EE(2))))<0.001 &&  EE(1)>min(P3(1),P4(1)) && EE(2)>min(P3(2),P4(2)) && EE(1)<max(P3(1),P4(1)) && EE(2)<max(P3(2),P4(2))
     disp 'INTERSECTING with Slope method'
     plot3([EE(1); EE2(1)],[EE(2);EE2(2)],[EE(3);EE2(3)])
    hold on
    pause(0.1)
    plot3([P3(1);P4(1)],[P3(2);P4(2)],[P3(3);P4(3)],'r')
    
    BySlopeMethod=((P4(2)-P3(2))*(P4(1)-EE(1)))-((P4(1)-P3(1))*(P4(2)-EE(2))) 
    
    pause()
end

hold off 
end

% Plot results here to verify intersection


