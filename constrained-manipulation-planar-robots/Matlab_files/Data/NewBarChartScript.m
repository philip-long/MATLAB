clear all,clc,close all 
DirList = dir(fullfile('./', '*bar_info.mat'));
for i=1:length(DirList)
    name_data{i}=DirList(i).name;
    Data{i} = load(fullfile('./', DirList(i).name));    
end
Eta=[]
Eta_r=[]
Eta_independant=[]
% Vol_1=[]
% Vol_2=[]
% Vol_ratio_1=[]
% Vol_ratio_2=[]
for i=1:length(Data)
   Eta=[Eta Data{i}.eta]
   Eta_r=[Eta_r Data{i}.eta_r]
   Eta_independant=[Eta_independant Data{i}.eta_small]
end


bar([Eta;Eta_r;Eta_independant])
ylim([0 1])
xlabel(['a','b','b'])


Eta_r
Eta_independant