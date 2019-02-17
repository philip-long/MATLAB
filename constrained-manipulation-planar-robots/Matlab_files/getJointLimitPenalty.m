function [ dmax,dmin ] = getJointLimitPenalty(q,qmax,qmin,n)
%getJointLimitPenalty Penalization function
%   Penalization function as configuration approaches joint limit
if(size(q,1)~=size(qmax,1))
    q=q';
end
qmean=(qmax+qmin)/2;
val=max(qmean,q)-qmean;

%d=exp((max(qmean,q)-qmax)/(qmean-qmax));
dmax=(((val)./((qmax-qmean)))).^n;

val=min(qmean,q)-qmean;
%d=exp((max(qmean,q)-qmax)/(qmean-qmax));
dmin=(((val)./((qmin-qmean)))).^n;

if (any(isnan(dmax)) || any(isinf(dmax)) || any(isnan(dmin)) || any(isinf(dmin)) )
    error('Joint position outside limits')
end
    
end

