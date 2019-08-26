function g = sigmoid(z,a)
%SIGMOID Compute sigmoid functoon
%   J = SIGMOID(z) computes the sigmoid of z.
if(nargin<2)
    a=1;
end
g = 1 ./ (1.0 + exp(-z*a));
end
