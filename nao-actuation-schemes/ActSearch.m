%%
clear all
clc
NaoParamsSerial
Counter=0;
global Qactuated Qpassive Qcut CfgActs
% Selects actuation scheme
Allschemes=nchoosek([1 2 3 4 5 6 7 8 9],4);
AllJoints=[1 2 3 4 5 6 7 8 9];
Qcut=10;
Cj=0;
Cga=0;
Cgp=0;
u=qinit;

disp 'Begin Iterations'


for i=1:length(Allschemes)
    
    
    
    Qactuated=Allschemes(i,:);
    Qpassive=AllJoints;
    
    ArchSing=0;
    for k =1:length(CfgActs)
        if all(ismember(Qactuated,CfgActs(k,:)))
            disp 'Scheme is Archetectural Singularity'
            disp 'Independent of Joint value'
            ArchSing=1;           
        end
    end
    
    
    if ArchSing==0 % Only if not in ArchSing
       
        
        for j=1:length(Qactuated)
            if any(ismember(Qpassive,Qactuated(j)))
                [r c]=find(Qpassive==Qactuated(j));
                Qpassive(c)=[];
            end
        end
        
        
        
        [Ga Gac Gp Gpc Gc]=ExtractGaGpGc(u);
        
        rank(Gp);
        
        
        if rank(Ga)<4
            %disp 'Ga losses rank'
            Cga=Cga+1;
            Garank(Cga,:)=Allschemes(i,:);
        end
        if rank(Gp)<5
            %disp 'Gp losses rank'
            Cgp=Cgp+1;
            Gprank(Cgp,:)=Allschemes(i,:);
            Gppass(Cgp,:)=Qpassive;
        else
            Jact=ActuatedJacobian(u);
            if rank(Jact)<4
                %disp 'Jact losses rank'
                Cj=Cj+1;
                Jactrank(Cj,:)=Allschemes(i,:);
            end
        end
        

    end
end