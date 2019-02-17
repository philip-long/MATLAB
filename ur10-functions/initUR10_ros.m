global R t rq

R{1}=RPY_to_Rot([0.0 0.0 0.0],"T");
t{1}=TransMat([0.0 0.0 0.1273]);
%rq{1}=[0 ;0 ;1];
rq{1}="z";

% upper_arm_link to shoulder_link
R{2}=RPY_to_Rot([0.0 1.570796325 0.0],"T");
quaternion2matrix_conROS([0.0 0.707 0.0 0.707]);
t{2}=TransMat([0.0 0.220941 0.0]);
%rq{2}=[0 ;1 ;0];
rq{2}="y";

% forearm_link to upper_arm_link
R{3}=RPY_to_Rot([0.0 0.0 0.0],"T");
t{3}=TransMat([0.0 -0.1719 0.612]);
%rq{3}=[0 ;1 ;0];
rq{3}="y";

% wrist_1_link to forearm_link
R{4}=RPY_to_Rot([0.0 1.570796325 0.0],"T");
t{4}=TransMat([0.0 0.0 0.5723]);
%rq{4}=[0 ;1 ;0];
rq{4}="y";

% wrist_2_link to wrist_1_link
R{5}=RPY_to_Rot([0.0 0.0 0.0],"T");
t{5}=TransMat([0.0 0.1149 0.0]);
rq{5}=[0 ;0; 1];
rq{5}="z";

% wrist_3_link to wrist_2_link
R{6}=RPY_to_Rot([0.0 0.0 0.0],"T");
t{6}=TransMat([0.0 0.0 0.1157]);
r{6}=[0 ;1 ;0];
rq{6}="y";

% ee_link to wrist_3_link
R{7}=RPY_to_Rot([0.0 0.0 1.570796325],"T");
t{7}=TransMat([0.0 0.0922 0.0]);
