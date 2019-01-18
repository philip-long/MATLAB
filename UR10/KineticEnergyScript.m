InitParams
% The Inertia Matrix Formalism gives:
global XX XY XZ YZ YY ZZ M MX MY MZ G3	
% Calculate the Inertia Matrix
q=[pi/4,pi/2,pi/3,pi/6,pi/4,pi/12];
A=A_ur10(q);
qdot=[0.5,0.1,0.2,0.55,0.1,0.8]';

E=0.5*(qdot'*A(1:6,1:6)*qdot)

%  Trying the Energy Formalism

% This returns the velocity of the links at the joint frame, but the linear and the angular velocity
Vr=velo_links(q,qdot);
strname="data/J"
for i = 1:7
	I_j{i}=[XX{i} XY{i} XZ{i}
                                  XY{i} YY{i} YZ{i}
                                  XZ{i} YZ{i} ZZ{i}];
                                  
                        MS{i}=[MX{i}; MY{i}; MZ{i}];
                                  
 	JJ{i}=[eye(3)*M{i},   -skew(MS{i})	;skew(MS{i}), I_j{i}	];
	
	E_k(i)=0.5*transpose(Vr{i})*JJ{i}*Vr{i};					
	
	  %filename=strcat(strname,num2str(i))
	  %fid = fopen (filename, "w");
	  %Name=strcat("Inertia Matrix of link ", num2str(i));
  	  %fdisp(fid,Name)
	  %fdisp(fid,size(JJ{i}))
	  %fdisp(fid,JJ{i})
	  %fclose(fid)
end
E_ki=sum(E_k)
E
# Using two different methods the difference is zero:
deltaE=E-E_ki

#  Conclusion: Given the correct inertial parameters I have two ways to calculate the kinetic energy, however I need:
#	
#	The inertia of each link in the joint frame
#	The centre of gravity of each link
#	The mass of each link
