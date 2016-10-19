%
clear all,clc
alpha=0.0;
i_ax=[1;0;0];
k_ax=[0;0;1];
j_ax=[0;1;0];

u1=[cos(alpha); -sin(alpha);0];
u2=[cos(alpha); sin(alpha);0];

theta=pi/6
ua=[cos(theta); sin(theta);0];
ub=[-cos(theta); -sin(theta);0];


zeta_A_i1=[zeros(3,1);i_ax];
zeta_A_i2=[zeros(3,1);j_ax];
zeta_A_03=[k_ax;cross(k_ax,ua)];
zeta_A_04=[cross(u1,k_ax);cross((cross(u1,k_ax)),ua)];

W_A=[zeta_A_i1 , zeta_A_i2 , zeta_A_03 ,zeta_A_04];



zeta_B_i1=[zeros(3,1);i_ax];
zeta_B_i2=[zeros(3,1);j_ax];
zeta_B_03=[k_ax;cross(k_ax,ub)];
zeta_B_04=[cross(u2,k_ax);cross((cross(u2,k_ax)),ub)];

W_B=[zeta_B_i1 , zeta_B_i2 , zeta_B_03 ,zeta_B_04];

W_C=[W_A W_B]
rank(W_C)


%%
clear all,clc

theta=-pi
while(theta<pi)
alpha=0;
i_ax=[1;0;0];
k_ax=[0;0;1];
j_ax=[0;1;0];
theta=theta+pi/19;
beta=rand(1);
Rot=expm(skew(j_ax)*beta);

%%
u1=[cos(alpha); -sin(alpha);0];
u2=[cos(alpha); sin(alpha);0];


uf=[cos(theta); -sin(theta);0];
ug=[-cos(theta); -sin(theta);0];


i_prime=Rot*i_ax;
k_prime=Rot*k_ax;

zeta_F_i1=[zeros(3,1);i_prime];
zeta_F_i2=[zeros(3,1);j_ax];
zeta_F_03=[k_ax;cross(k_ax,uf)];
zeta_F_04=[cross(u1,k_ax);cross((cross(u1,k_ax)),uf)];

W_F=[zeta_F_i1 , zeta_F_i2 , zeta_F_03 ,zeta_F_04];


zeta_G_i1=[zeros(3,1);i_prime];
zeta_G_i2=[zeros(3,1);j_ax];
zeta_G_03=[k_ax;cross(k_ax,ug)];
zeta_G_04=[cross(u2,k_ax);cross((cross(u2,k_ax)),ug)];

W_G=[zeta_G_i1 , zeta_G_i2 , zeta_G_03 ,zeta_G_04];


W_C=[W_F W_G];
rank(W_C)
pause()
endwhile