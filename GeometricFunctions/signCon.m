function a=signCon(x)
% signCon which is defined as 1 when input is zero;
s=sign(x);
a=s;
a(s==0)=1;
end
