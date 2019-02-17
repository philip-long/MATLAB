function plotPoint( p,format )
%plotPoint simple plot point function
if(nargin==1)
    format='ko';
end

if(max(size(p)==2))
    plot(p(1),p(2),format);
elseif(max(size(p)==3))
    plot3(p(1),p(2),p(3),format);
end

end

