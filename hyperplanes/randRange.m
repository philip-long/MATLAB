function [ r] = randRange(upper,lower,n)
%randRange Generate a random vector of n elements in a ranges. 
if(nargin<3)
    n=1;
end
r=lower+(upper-lower)*rand(1,n);

end

