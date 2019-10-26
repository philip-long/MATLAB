%% Gradient Check of normalized cross product for a set of vectors
clear all,clc

cross_product=zeros(3,1);
norm_of_cross_product=zeros(3,1);
norm_single_vector=0;
n=zeros(3,1);

q=-2;
step=0.0001;
NUM=[];
ANALYTIC=[];
E=[]
for q=-rand(1)*4:step:rand(1)*4
    
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
    dndq= (dudq*v - u*dvdq )/ v^2;
    numerial_grad_n=(n-n_last)/step;
    
    
    
    NUM=[NUM;numerial_grad_n'];
    ANALYTIC=[ANALYTIC;dndq'];
    
    E=[E;norm(numerial_grad_n-dndq)];
end
plot(E(2:end))
pause()
for i=1:3
    plot(NUM(2:end,i),'ro');
    hold on
    plot(ANALYTIC(2:end,i),'b');
end


%% Gradient Check of normalized cross product for a set of jacobian columns
clear all,clc

cross_product=zeros(3,1);
norm_of_cross_product=zeros(3,1);
norm_single_vector=0;
n=zeros(3,1);

qpos=randRange(-pi,pi,7);
joint=randi(7);
%joint=1;
T=T70(qpos);
J=J70(qpos);
S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
JE=S*J;
v1=JE(1:3,1);
v2=JE(1:3,2);
joint;
E=[];
NUM=[];
ANALYTIC=[];
step=0.0001;

for qjoint=0:step:pi
    
    qpos(joint)=qjoint;
    T=T70(qpos);
    J=J70(qpos);
    S=screwTransform(T(1:3,1:3)*[0.55;0.0;0.0]);
    JE=S*J;
    v1=JE(1:3,1);
    v2=JE(1:3,4);
    H=getHessian(JE);
    dv1=H{joint}(1:3,1);
    dv2=H{joint}(1:3,4);
    

    
    
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
    dndq= (dudq*v - u*dvdq )/ v^2;
    numerial_grad_n=(n-n_last)/step;
    
    dndq=getGradientn(v1,v2,dv1,dv2);
    
    
    NUM=[NUM;numerial_grad_n'];
    ANALYTIC=[ANALYTIC;dndq'];

    E=[E;norm(numerial_grad_n-dndq)];

end
plot(E(2:end))
pause()


for i=1:3
    plot(NUM(2:end,i),'ro');
    hold on
    plot(ANALYTIC(2:end,i),'b');
end
