clear all
% Dimension of UR5
D3=0.425;
D4=0.392;
RL2=0.109;
RL5=0.094;
% Modified DH parameters
sigma = [0 0 0 0 0 0];
alpha = [0 -pi/2 0 0 pi/2 -pi/2];
d = [0 0 D3 D4 0 0];
theta = [0 0 0 0 0 0];
r = [0 RL2 0 0 RL5 0];

% Generating various q within the joint limits
q=-3.14+6.28.*rand(6,1)

% Calling the IGM function for the possible solutions
[q1,q2,q3,q4,q5,q6,q7,q8] = IGMUR5(sigma,alpha,d,theta,r,q)

% Checking the solutions obtained
 T0Tn  = GENDGM(sigma,alpha,d,theta,r,q) % The value of q generated randomly
 T0Tn1 = GENDGM(sigma,alpha,d,theta,r,q1) % Solution1
 T0Tn2 = GENDGM(sigma,alpha,d,theta,r,q2) % Solution2
 T0Tn3 = GENDGM(sigma,alpha,d,theta,r,q3) % Solution3
 T0Tn4 = GENDGM(sigma,alpha,d,theta,r,q4) % Solution4
 T0Tn5 = GENDGM(sigma,alpha,d,theta,r,q5) % Solution5
 T0Tn6 = GENDGM(sigma,alpha,d,theta,r,q6) % Solution6
 T0Tn7 = GENDGM(sigma,alpha,d,theta,r,q7) % Solution7
 T0Tn8 = GENDGM(sigma,alpha,d,theta,r,q8) % Solution8