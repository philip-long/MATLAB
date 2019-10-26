
%% Now try the normalized vectors
clear all,clc,close all

cross_product=zeros(3,1);
norm_of_cross_product=zeros(3,1);
norm_single_vector=0;
n=zeros(3,1);
nt_vk=zeros(3,1);
sig_nt_vk=zeros(3,1);
hp=zeros(3,1);
q=-2;
step=0.0001;

deltaq=rand(1)*2;
E=[]
for q=-2:step:2

v1=[q^4 + 6*q;-6*q^2 + 5*q; 1*q^2];
dv1=[(4*q^3)+6;-12*q+5;2*q];

v2=[q^3 + 2*q;-2*q^3; -8*q];
dv2=[3*q^2+2;-6*q^2;-8];    

vk=[0.1*q^2 + 1*q;-0.2*q^2; 0.1*q^4];
dvk=[0.2*q+1;-0.4*q;.4*q^3];   
    
cross_product_last=cross_product;
norm_single_vector_last=norm_single_vector;
norm_of_cross_product_last=norm_of_cross_product;
n_last=n;
nt_vk_last=nt_vk;
sig_nt_vk_last=sig_nt_vk;
h_last=hp;

% Derivate of nt Checked
u=cross(v1,v2);
v=norm(cross(v1,v2));

dudq=-skew(v2)*dv1 + skew(v1)*dv2;
dvdq=(dudq'*u)/((u'*u)^0.5);

n=u/v;
dndq= (dudq*v - u*dvdq )/ v^2;
numerial_grad_n=(n-n_last)/step;

nt_vk=n'*vk;
%  gradient of nt*vk CHECKED
dntvk_dq=(dndq'*vk) +n'*dvk;
numerial_grad_ntvk=(n'*vk-nt_vk_last)/step;


% GRADIENT OF SIGMOID(nt*vk) CHECKED
sig_nt_vk=sigmoid(nt_vk,200);
dsig_nt_vk_dq=sigmoidGradient(nt_vk,-200)*dntvk_dq;
numerial_grad_sig_nt_vk=(sig_nt_vk-sig_nt_vk_last)/step;

hp=sigmoid(nt_vk,-200)*deltaq*nt_vk;
hm=sigmoid(nt_vk,-200)*deltaq*nt_vk;

% Gradient of H CHECKED
dhdq=(dsig_nt_vk_dq*deltaq*nt_vk) + sigmoid(nt_vk,-200)*deltaq*dntvk_dq;
numerical_gradient_h=(hp-h_last)/step;
numerical_gradient_h=(hp-h_last)/step;

% if(dsig_nt_vk_dq>0.001)
% pause()
% end
    E=[E;norm(numerical_gradient_h-dhdq)];
end
plot(E(2:end))
figure(2)
plot(Em(2:end))