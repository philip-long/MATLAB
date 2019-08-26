% gradient of cross product test

cv1v2=zeros(3,1);
v1=zeros(3,1);
v2=zeros(3,1);
step=0.001;
Eg=[]
Ev1=[]
Ev2=[]
for q=-2:step:2

cv1v2_last=cv1v2;
v1_last=v1;
v2_last=v2;

v1=[q^2 + 6*q;-6*q^2 + 12*q; 1*q];
dv1=[2*q+6;-12*q+12;1];
v2=[q^3 + 2*q;-2*q; -8*q];
dv2=[3*q^2+2;-2;-8];

dc=-skew(v2)*dv1 + skew(v1)*dv2
dc2=cross(dv1,v2)+cross(v1,dv2)


cv1v2=cross(v1,v2);

grad_cross=(cv1v2-cv1v2_last)/step;
grad_v1=(v1-v1_last)/step;
grad_v2=(v2-v2_last)/step;


grad_cross-dc
Eg=[Eg;norm(grad_cross-dc)];
Ev1=[Ev1;grad_v1-dv1];
Ev2=[Ev2;grad_v2-dv2];

end

plot(Eg(2:end),'r');
hold on
plot(Ev1(4:end),'b');
plot(Ev2(4:end),'g');
%% Now try the normalized vectors
clear all,clc

cross_product=zeros(3,1);
norm_of_cross_product=zeros(3,1);
norm_single_vector=0;
n=zeros(3,1);

q=-2;
step=0.001;
for q=-2:step:2

v1=[q^4 + 6*q;-6*q^2 + 5*q; 1*q^2];
dv1=[(4*q^3)+6;-12*q+5;2*q];

v2=[q^3 + 2*q;-2*q^3; -8*q];
dv2=[3*q^2+2;-6*q^2;-8];    
    
cross_product_last=cross_product;
norm_single_vector_last=norm_single_vector;
norm_of_cross_product_last=norm_of_cross_product;
n_last=n;




 % derivative of cross product CHECKED this is fine 
 cross_product=cross(v1,v2);
dc=-skew(v2)*dv1 + skew(v1)*dv2;
grad_cross=(cross_product-cross_product_last)/step;



% So I need to get the derivative of the norm of the cross product

% norm of vector derivative CHECKED this is fine 
norm_single_vector=norm(v1);
% (dv1'*v1 + v1'*dv1) / (2*(v1'*v1)^0.5) pre-simplification
analytical_diff_norm=(dv1'*v1)/((v1'*v1)^0.5);
grad_norm=(norm_single_vector-norm_single_vector_last)/step;
 
% norm of the cross project  CHECKED this is fine 
norm_of_cross_product=norm(cross(v1,v2));
u1=cross(v1,v2);
du1dq=-skew(v2)*dv1 + skew(v1)*dv2;
analytical_diff_cross_norm=(du1dq'*u1)/((u1'*u1)^0.5);
grad_cross_norm=(norm_of_cross_product-norm_of_cross_product_last)/step;



% cross product divided by its norm amazingly this actually works fine
% n = cross(v1,v2) / norm(cross(v1,v2))
% n=   u/v
% dn  =  du v - u dv
% dq     dq       dq
%       --------------- 
%             v'*v

u=cross(v1,v2);
v=norm(cross(v1,v2));

dudq=-skew(v2)*dv1 + skew(v1)*dv2;
dvdq=(dudq'*u)/((u'*u)^0.5);

n=u/v;
dndq= (dudq*v - u*dvdq )/ v^2
numerial_grad_n=(n-n_last)/step
pause()





end

