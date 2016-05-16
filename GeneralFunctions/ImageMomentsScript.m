% In this script I will see if can do the image
% thing in Matlab
Aimage=imread('TestImages\TestImage1.bmp');
for i=1:size(Aimage,1)
    for j=1:size(Aimage,2)
        if Aimage(i,j)>0
            Aimage(i,j)=1;
            B(i,j)=double(Aimage(i,j));
        end
    end
end

A=Aimage;
    %%


imshow(A)
s=regionprops(A,'Area','Centroid','Orientation');
s=regionprops(A)


% Define the moments; now its starting to make
% sense

%%
A=B;
m00=raw_moments(A,0,0);
m10=raw_moments(A,1,0);
m01=raw_moments(A,0,1);
m11=raw_moments(A,1,1);
mu00=cent_moment(0,0,A);
mu10=cent_moment(1,0,A);
mu01=cent_moment(0,1,A);
mu11=cent_moment(1,1,A);
mu21=cent_moment(2,1,A);
mu12=cent_moment(1,2,A);
mu02=cent_moment(0,2,A);
mu20=cent_moment(2,0,A);
mu03=cent_moment(0,3,A);
mu30=cent_moment(3,0,A);
mu22=cent_moment(2,2,A);
mu05=cent_moment(0,5,A);
mu32=cent_moment(3,2,A);
mu23=cent_moment(2,3,A);
mu14=cent_moment(1,4,A);
mu41=cent_moment(4,1,A);
mu50=cent_moment(5,0,A);



[M]=feature_vec(A)

% Invariant moments by hand cals
I1=-cent_moment(2,0,A)*cent_moment(0,2,A)+(cent_moment(1,1,A))^2

I13=(mu50+(2*mu32)+mu14)^2 + ( mu05 + (2*mu23)+mu41)^2;
I14=(mu50-(2*mu32)-(3*mu14))^2 + ( mu05 - (2*mu23)-(3*mu41))^2;
I15=(mu50-(10*mu32)+ (5*mu14))^2 + ( mu05 - (10*mu23)+(5*mu41))^2;
% Define the image features xg yg a , tw xi alpha
xg=m10/m00;
yg=m01/m00;
a=m00;
alpha=0.5*(atan((2*mu11/(mu20-mu02))))
c9=I13/I15;
c10=I14/I15;

% Now we define a new set of feature
Zd=100;ad=100;
an=Zd*(ad/m00)^0.2;
yng=an*(m01/m00);
xng=an*(m10/m00);
alpha=0.5*(atan((2*mu11/(mu20-mu02))));
c9=I13/I15;
c10=I14/I15;

% Intermidiate variables
eta11=mu11/m00;
eta20=mu20/m00;
eta02=mu02/m00;
epp11=4*eta11 - xg*yg*0.5;
epp22=epp11;
epp12=4*eta20 - xg*xg*0.5;
epp21=4*eta02 - yg*yg*0.5;
epp31=3*yg*0.5;
epp32=3*xg*0.5;

Beta=5;Gamma=1;
d=(mu20-mu02)^2 + 4*(mu11)^2;
alphawx=(Beta*(mu12*(mu20-mu02) + mu11*(mu03-mu21))  + Gamma*xg*(mu02*(mu20-mu02)-2*(mu11)^2) +Gamma*yg*mu11*(mu20+mu02))/d;
alphawy=(Beta*(mu21*(mu02-mu20) + mu11*(mu30-mu12))  + Gamma*xg*mu11*(mu20+mu02)   +Gamma*yg*(mu20*(mu02-mu20)-2*(mu11)^2) )/d;
% Now write the interaction matrix

Lxn=[-1 0 0 an*epp11  -an*(1+epp12) yng]
Lyn=[0 -1 0 an*(1+epp21) -an*epp22 -xng]
Lan=[0 0 -1 -an*epp31 an*epp32 0]
Lci=[0 0 0 ciwx ciwy 0]
Lcj=[0 0 0 cjwx cjwy 0]
Lalpha=[0 0 0 alphawx alphawy -1]




% Trying to derive alphawx and alpha wy in fact
% Tahri 2009 Point based and region has a couple
% of errors so be careful
% clear all
% syms xg yg DELTA mu11 mu20 mu02 mu03 mu30 mu12 mu21
% 
% 
% a=mu11*(mu20+mu02)/DELTA;
% b=(2*(mu11^2)+mu02*(mu02-mu20))/DELTA;
% c=(2*(mu11^2)+mu20*(mu20-mu02))/DELTA;
% d=5*(mu12*(mu20-mu02) + mu11*(mu03-mu21))/DELTA
% e=5*(mu21*(mu02-mu20)+mu11*(mu30-mu12))/DELTA
% 
% alphawx2=-b*xg+a*yg+d
% alphawy2=a*xg-c*yg+e


% Now I have to derive ci and cj actually I have
% to derive the interaction matrix associated