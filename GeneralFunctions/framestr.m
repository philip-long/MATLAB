function [ out ] = framestr( x, tam )

 ceros=strrep( num2str(zeros(1,tam-length(num2str(x)))) ,' ','');
 
 out = [ceros num2str(x)];

end
