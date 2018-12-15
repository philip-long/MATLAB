function D= distPoint2Points( Point1,Points)
%distPoints Distance between  point1 and a matrix of other points
%   Point1 a 1x3 or 1x2 vector
%   Points a nx3 or nx2 vector
%   one arguement : distance to origin
%   two arguments : planar distance between points
%   three arguments : spatial distance between points
D=zeros(size(Points,1),1);
for i=1:size(Points,1)    
    if(length(Point1)==2)
        d=distPoints([Points(i,1),Point1(1)],[Points(i,2),Point1(2)]);
        D(i)=d;
    elseif(length(Point1)==3)
        d=distPoints([Points(i,1),Point1(1)],[Points(i,2),Point1(2)],[Points(i,3),Point1(3)]);
        D(i)=d;
    end
    
end


