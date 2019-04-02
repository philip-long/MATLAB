function ST_i=invTransScrew(S)
% Converts a twist screw transform to a wrench screw transform
% vi = (S x vj)
% fi = (ST_i x fj)
ST_i=S;
ST_i(1:3,4:6)=S(4:6,1:3);
ST_i(4:6,1:3)=S(1:3,4:6);

end
