function R=angleAxis_2Rot(u,theta)


R=expm(skew(u/norm(u))*theta);
