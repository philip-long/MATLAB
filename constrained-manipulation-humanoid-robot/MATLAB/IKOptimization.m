function [exitflag_init,good_solution]=IKOptimization(constrained,object_case,initial_condition,solver_name)
% IKOptimization: Obtain an inverse kinematic solution for a desired pose in a cluttered environment


[RobotChain,gains]=getRobotParameters();
object_constants=getObjectParams(object_case); % CHANGES THE OBJECT CASE

%%
%roslaunch numl_val_motion_planning val_model.launch
%
try
    joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
catch
    rosinit
    joint = rossubscriber('/dummy_joint_states','sensor_msgs/JointState');
end
val=importrobot('valkyrie_sim.urdf');
val.DataFormat='column';
r=importrobot('valkyrie_sim.urdf');

%%
Q = receive(joint,5.0);
q_config=homeConfiguration(r);
q=zeros(32,1);
qnames=cell(32,1);

for i=1:length(q_config)
    for j=1:length(Q.Name)
        if(all(strcmp( q_config(i).JointName,Q.Name(j))))
            q_config(i).JointPosition=Q.Position(j);
            q(i)=Q.Position(j);
            qnames(i)=Q.Name(j);
        end
    end
end


%% Formulate an IK problem that takes joint variables and lf_T_opt
% This finds the best posture to maximize the manipulability polytope
%

tinit=initial_condition;
qinit=(ones(length(q),size(initial_condition,1)).*q)';
cpoints=CustomStartPointSet([ initial_condition,qinit]);
op_P_d=[0.25,0.3,0.9]';

tic
fmincon_options= optimoptions('fmincon',...
    'Algorithm',solver_name,...
    'OptimalityTolerance',1.0,...
    'TolCon',0.001,...
    'StepTolerance',0.00001);

x_op1=tinit(1);
y_op1=tinit(2);
theta_op1=tinit(3);
% Decision variables are leftfoot pose and joint variables, these values are ignored
% but given fmincon dimension of problem.
x0=[x_op1;y_op1;theta_op1;q]; 


if(constrained)
    fun = @costWpStar;
else
    fun = @costWp;
end
nonlcon=@nlconPose;
A=[];
b=[];
Aeq=[];
beq=[];
ub=zeros(35,1);
lb=zeros(35,1);

lb(1:3)=[-0.1,-1.0,-pi];
ub(1:3)=[1.0,0.8,pi];
for i=1:length(qnames)
    for j=1:length(RobotChain.JOINT_NAMES)
        if(strcmp(qnames(i),RobotChain.JOINT_NAMES(j)))
            ub(3+i)=RobotChain.JOINT_POSITION_MAX(j);
            lb(3+i)=RobotChain.JOINT_POSITION_MIN(j);
        end
    end
end

%[x,fval_init,exitflag_init,output_init,lambda]=fmincon(@(x)fun(x,val,q,qnames,RobotChain,object_constants,gains),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(op_P_d,x,val),fmincon_options);


problem = createOptimProblem('fmincon','objective',@(x)fun(x,val,q,qnames,RobotChain,object_constants,gains),...
                                            'x0',x0,'Aineq',A,'bineq',b,'Aeq',Aeq,'beq',beq,...
                                            'lb',lb,'ub',ub,...
                                            'nonlcon',@(x)nonlcon(op_P_d,x,val),...
                                            'options',fmincon_options);
                                        
                                        
ms=MultiStart('Display','iter','UseParallel',1)
tic
[x,fval_init,exitflag_init,outpt,allmins] = run(ms,problem,cpoints);
toc

good_solution=0;
for k=1:length(allmins)
    if(allmins(k).Exitflag>0)
        good_solution=good_solution+1;
    end
end
%figure(2)

