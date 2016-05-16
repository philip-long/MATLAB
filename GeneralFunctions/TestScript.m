% 




Y_Measure=[];
W_observe=[];
% Tool Frame Offset
Mx=0.0;My=0.0;Mz=0.14; 
fMt=[0.0 1 0 Mx;-1 0 0 My; 0 0 1 Mz; 0 0 0 1];

while(1) % The online calibration script

bMt=TransMat(pi/4,'rx','T')*TransMat(pi/5,'rz','T')*TransMat(rand(1),'rx','T')*TransMat(rand(3,1),'T');
fMb=fMt*inv(bMt);
MeasuredForce=rand(6,1);

Bsim=[eye(3) zeros(3) -9.81*fMb(1:3,3) -9.81*fMb(1:3,3) zeros(3)
     zeros(3) eye(3) zeros(3,2) -9.81*skew(fMb(1:3,3))] ;

Y_Measure=[Y_Measure; MeasuredForce]
W_observe=[W_observe;Bsim]
pause()

end
x=pinv(W_observe)*Y_Measure