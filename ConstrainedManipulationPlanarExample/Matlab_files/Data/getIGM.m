function configs=getIGM(Xd,Yd)
l1=0.8;
l2=0.55;

Z1=Xd;
Z2=Yd;
X=l1;
Y=l2;

CTj=((Z1*Z1)+(Z2*Z2)-(X*X)-(Y*Y) )/(2*X*Y);
try   
    Tj{1}=atan2(sqrt(1-CTj*CTj),CTj);
    Tj{2}=atan2(-sqrt(1-CTj*CTj),CTj);
    Ti={};
    for i=1:2
        theta_j=Tj{i};
        B1=X+Y*cos(theta_j);
        B2=Y*sin(theta_j);
        STi=(B1*Z2 - B2*Z1) / (B1*B1 + B2*B2);
        CTi=(B1*Z1 + B2*Z2) / (B1*B1 + B2*B2);
        Ti{i}=atan2(STi,CTi);
    end
    configs=[Ti{1} Tj{1}
            Ti{2} Tj{2}];
   % So I want the first config to be elbow down config
    if(configs(1,2)<0.0)
        a=configs;
        configs(1,:)=a(2,:);
        configs(2,:)=a(1,:);
           configs
        pause()  
    end
catch
    disp 'out of workspace'
    configs=0;
end



end