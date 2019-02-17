function [ dmax,dmin ] = getJointLimitPenalty(q,qmax,qmin,n)
%getJointLimitPenalty get the penalty function as a joint approaches its
%position limits

if(size(q,1)~=size(qmax,1))
    q=q';
end
qmean=(qmax+qmin)/2;
val=max(qmean,q)-qmean;


dmax=(((val)./((qmax-qmean)))).^n;

val=min(qmean,q)-qmean;
dmin=(((val)./((qmin-qmean)))).^n;

if (any(isnan(dmax)) || any(isinf(dmax)) || any(isnan(dmin)) || any(isinf(dmin)) )
    error('Joint position outside limits')
end
    
end

