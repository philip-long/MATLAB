% Danger Grid normalisations

clear all,clc,close all


d=0.001:0.01:10;
min_dis=0.01;
max_dis=5.0;
D=[]
for i=1:length(d)
     if((d(i))<min_dis)
    D=[D 1/min_dis];
    elseif((d(i))>max_dis)
    D=[D 1/max_dis];
    else
     D=[D 1/d(i)];
   endif
 
endfor

Dn= ( D-min(D) ) ./  ( max(D)-min(D) ) ;

plot(d,Dn,'r','LineWidth',5.0)
close all

P=[0.0;0.0]
Pc=[-1.0;0.2]
Velocity_robot_point=[0.0;0.33]
RDIFF=[];
DANGER=[];
 plot(P(1),P(2),'bo',"MarkerSize",5.0,"LineWidth",5.0)
    hold on
    max_speed=0.25
for i=1:1:20
  Pc(1)=Pc(1)+0.1;
   r_diff=Pc-P;     
   norm_r_diff=norm(r_diff);
   cos_angle= (r_diff'*Velocity_robot_point(1:2)) / (norm_r_diff*norm(Velocity_robot_point(1:2)))
   Danger=norm(Velocity_robot_point(1:2))*cos_angle;
   
    if(Danger<0.0)
    Danger=0.0;
    elseif(Danger>max_speed)
    Danger=max_speed;
   endif
   
   RDIFF=[RDIFF r_diff];
   DANGER=[DANGER Danger];
   plot(r_diff(1),r_diff(2),'rx',"MarkerSize",5.0,"LineWidth",5.0)

endfor

DANGERp= ( DANGER-min(DANGER) ) ./  (max_speed-min(DANGER) ) ;

figure(2)
plot(1:1:20,DANGERp)





% Basically the danger field will be calculated for each cell 
% as 
% for each robot point:
%        distance to point AND if that point is moving towards operator AND
%        if the normalized interia thing is significant
%        for each parameter we add in a min & max value such that minimum
%        value is 0 and maximum value is 1
%        then we calculate total danger as a product, phrased
%   It is (dangerous) because:
%            1. We are realtively close to the robot
%   AND      2. The robot is moving at a high speed towards us
%   AND      3. The normalized impulse vector in this direction is high


% Afterwards we can compare this to probability that someone will be present
% and feed both or all index to the congintive block

% TODO GET normalized impulse vector working

% the result is something between a danger field and a danger index