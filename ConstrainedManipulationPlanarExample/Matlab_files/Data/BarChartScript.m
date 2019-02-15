clear all,clc,close all 
C12_bar_info
DirList = dir(fullfile('./', '*bar_info.mat'));
for i=1:length(DirList)
    name_data{i}=DirList(i).name;
    Data{i} = load(fullfile('./', DirList(i).name));    
end
Q_ratio_1=[]
Q_ratio_2=[]
% Vol_1=[]
% Vol_2=[]
% Vol_ratio_1=[]
% Vol_ratio_2=[]
for i=1:length(Data)
    if(name_data{i}(2)=='1')
        Q_ratio_1=[Q_ratio_1 (Data{i}.q_vold)/(Data{i}.q_vol)];
        Vol_1=[Vol_1 (Data{i}.volCd(2))];
        Vol_ratio_1=[Vol_ratio_1 (Data{i}.volCd(2))/(Data{i}.volC(2))];
    elseif(name_data{i}(2)=='2')
        Q_ratio_2=[Q_ratio_2 (Data{i}.q_vold)/(Data{i}.q_vol)];
        Vol_2=[Vol_2 (Data{i}.volCd(2))];
        Vol_ratio_2=[Vol_ratio_2 (Data{i}.volCd(2))/(Data{i}.volC(2))];
    end
end
Q=[],V=[],Vr=[]
for i=1:3
    Q=[Q, Q_ratio_1(i) Q_ratio_2(i)];
    V=[V,Vol_1(i) Vol_2(i)];
    Vr=[Vr, Vol_ratio_1(i) Vol_ratio_2(i)];
end
bar([Q;V])
