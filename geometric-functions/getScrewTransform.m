function [L l]=getScrewTransform(T,d)
% getLinkOffsets Returns a cell containing all offsets so that ellipsoids are represented at center of link 


l=T(1:3,1:3)*d;
Lhat=skew(l);
L=[eye(3) -Lhat; zeros(3) eye(3) ];        


% %%
% 
% %In order to find the Jacobian matrix at the tool frame rather than the last joints
% q=qinit
% T= T70(q);% Transformation matrix 
% J= J70(q); 
% P=TnE(1:3.4); %Distance to tool frame
% L=T(1:3.1:3)*P; %Change the reference frame to the world one  
% Lhat=skew(L); %Skew the matrix
% 
% J0e=[eye(3) -Lhat; zeros(3) eye(3) ]*J; % Change the point of the Jacobian