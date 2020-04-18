function uskew=skew(u)

%This function gets the skew symmtric matrix that preforms the cross 
%product via mutiplication

if length(u)==3
    uskew=[0   -u(3)     u(2)
           u(3) 0       -u(1)
          -u(2) u(1)     0];
else
    disp 'Only works for vectors of 3  elements'
end