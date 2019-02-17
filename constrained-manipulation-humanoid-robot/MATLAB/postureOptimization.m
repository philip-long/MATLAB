
function initial_condition_ok=postureOptimization(constrained,object_case,initial_condition,test_no,solver_name)
%postureOptimization Obtain the posture of the robot to mazimize some
%criteria for a given left palm desired pose in a cluttered environment. 
% 

[RobotChain,gains]=getRobotParameters();
object_constants=getObjectParams(object_case); % CHANGES THE OBJECT CASE


file_name=int2str(test_no);

if(constrained)
    file_name=strcat('wpstar_t',file_name,solver_name,'obj_',int2str(object_case),'_',date);
else
    file_name=strcat('wp_t',file_name,'obj_',int2str(object_case),'_',date);
end

initial_condition_ok=true;
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

%% Objects, plot world
%lfTp=eye(4);lfTp(1,4)=0.55; lfTp(2,4)=-0.2;lfTp(3,4)=-0.090;


tinit=initial_condition;
lf_T_opt=constructOpFrame(tinit);
objects=getObjectsLeftFootFrame(lf_T_opt,object_constants);
% showObjects(objects);

op_P_d=[0.25,0.3,0.9]';
% lf_P_d=lf_T_opt*[op_P_d;1];
% publishControlPoint(lf_P_d,1,'leftFoot');

d=100;
% Check initial condition
for l=1:length(val.BodyNames)
    for obj = 1:length(objects)
        try
            d1=getMinObjectDistance(val,q,val.BodyNames{l},objects{obj});
            d=min(d,d1);
        catch
            d=0.0;
        end
    end
end

if(d<0.01) 
    initial_condition_ok=false;
    return 
else
    disp 'Initial Condition Collision Free'
end



%% STEP 1 Formulate an IK problem that takes joint variables and lf_T_opt
% This finds the best posture to maximize the manipulability polytope
%
tic
fmincon_options= optimoptions('fmincon',...
    'Algorithm',solver_name,...
    'OptimalityTolerance',5.0,...
    'TolCon',0.001,...
    'StepTolerance',0.00001);

x_op1=tinit(1);
y_op1=tinit(2);
theta_op1=tinit(3);
x0=[x_op1;y_op1;theta_op1;q]; % Decision variables are leftfoot pose and joint variables


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

[x,fval_init,exitflag_init,output_init,lambda]=fmincon(@(x)fun(x,val,x0(4:end),qnames,RobotChain,object_constants,gains),...
    x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(op_P_d,x,val),fmincon_options);
x_lf=x(1:3);
x_joints=x(4:end);
toc

if(exitflag_init==-1 || exitflag_init==-2)
    disp ('Initial_optimization failed');
    initial_condition_ok=true;    
    file_name=strcat('Fail',file_name);
    save(file_name)
    return
end
%% Workspace Analysis with the resulting posture in the volume around the desired pose try to find feasible solutions.
% interior-point

fmincon_options2= optimoptions('fmincon',...
    'Algorithm',solver_name,...
    'TolCon',0.001);


x0=x_joints;
lf_T_op=constructOpFrame(x_lf);
lf_objects=getObjectsLeftFootFrame(lf_T_op,object_constants);
lfTrfd=getTransform(val,x_joints,'rightFoot');


A=[];
b=[];
Aeq=[];
beq=[];
ub=zeros(32,1);
lb=zeros(32,1);



for i=1:length(qnames)
    for j=1:length(RobotChain.JOINT_NAMES)
        if(strcmp(qnames(i),RobotChain.JOINT_NAMES(j)))
            ub(i)=RobotChain.JOINT_POSITION_MAX(j);
            lb(i)=RobotChain.JOINT_POSITION_MIN(j);
        end
    end
end

POS=[];
COST=[];
DIST=[];
EXITFLAG=[];
WPSTAR=[];
FVAL=[];
Q_CONFIG={};
cell_ind=1;

VERTICES=[op_P_d(1)-0.1,op_P_d(1)+0.09
    op_P_d(2)-0.1,op_P_d(2)+0.1
    op_P_d(3)-0.08,op_P_d(3)+0.1];

Avertices=[eye(3);-eye(3)];
bvertices=[VERTICES(1:3,2);-VERTICES(1:3,1)];
Desired_Vertices=Polyhedron(Avertices,bvertices);

lf_Pd_cell=zeros(3,1);

for a=1:size(Desired_Vertices.V,1)
    temp=lf_T_op*[Desired_Vertices.V(a,:)';1];
    lf_Pd_cell(1)=temp(1);
    lf_Pd_cell(2)=temp(2);
    lf_Pd_cell(3)=temp(3);
   
    [xwk,fval,exitflag,output,lambda]=fmincon(@(xwk)costWrkspace(xwk,val,x0,lf_Pd_cell,RobotChain),...
        x0,A,b,Aeq,beq,lb,ub,...
        @(xwk)nloconWrkSpace(xwk,val,lfTrfd,lf_objects,RobotChain),fmincon_options2);

    wp=getManipulability(val,xwk,qnames,RobotChain );
    wpstar=getConstrainedManipulability(val,xwk,qnames,lf_objects,RobotChain,gains);
    

    d=100;
   
    for l=1:length(val.BodyNames)
        for obj = 1:length(objects)
            try
                d1=getMinObjectDistance(val,xwk,val.BodyNames{l},objects{obj});
                d=min(d,d1);
            catch
                d=0.0;
            end
        end
    end
    
    POS=[POS;lf_Pd_cell'];
    COST=[COST;wp];
    WPSTAR=[WPSTAR;wpstar.volume];
    DIST=[DIST;d];
    FVAL=[FVAL;fval];
    EXITFLAG=[EXITFLAG;exitflag];
    Q_CONFIG{cell_ind}=xwk;
    cell_ind=cell_ind+1
end
notifySend('I am finished');
save(file_name)
end

