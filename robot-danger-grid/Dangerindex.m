

clear all,clc


% Solve for the power later
minimum_value=0.0;
maximum_value=1.0;

cell_distance=0.01:0.1:6;
DI=1./(cell_distance).^2;

for i=1:length(DI) 
  if(DI(i)<minimum_value)
  DI(i)=minimum_value;  
  elseif(DI(i)>maximum_value)
  DI(i)=maximum_value;
  endif
  endfor
  
Position_index=(DI-minimum_value)/(maximum_value-minimum_value);
plot(cell_distance,Position_index,'LineWidth',5.0)


% Speed