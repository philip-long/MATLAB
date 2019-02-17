function [RobotChain,gains]=getRobotParameters()
% getRobotParameters returns all robot parameters

RIGHT_ARM={    'rightShoulderPitch'
    'rightShoulderRoll'
    'rightShoulderYaw'
    'rightElbowPitch'
    'rightForearmYaw'
    'rightWristRoll'
    'rightWristPitch'};
LEFT_ARM={    'leftShoulderPitch'
    'leftShoulderRoll'
    'leftShoulderYaw'
    'leftElbowPitch'
    'leftForearmYaw'
    'leftWristRoll'
    'leftWristPitch'};
LEFT_ARM_LINKS={    'leftShoulderPitchLink'
    'leftShoulderRollLink'
    'leftShoulderYawLink'
    'leftElbowPitchLink'
    'leftForearmLink'
    'leftWristRollLink'
    'leftPalm'};

NECK={    'lowerNeckPitch'
    'neckYaw'
    'upperNeckPitch'};
TORSO={    'torsoYaw'
    'torsoPitch'
    'torsoRoll'};
RIGHT_LEG={'rightHipYaw'
    'rightHipRoll'
    'rightHipPitch'
    'rightKneePitch'
    'rightAnklePitch'
    'rightAnkleRoll'};
LEFT_LEG={'leftAnkleRoll'
    'leftAnklePitch'
    'leftKneePitch'
    'leftHipPitch'
    'leftHipRoll'
    'leftHipYaw'};


RIGHT_ARM_MAX_POSITION_LIMITS=[2.0,1.51,2.18,2.174,3.14,0.62,0.36];
LEFT_ARM_MAX_POSITION_LIMITS=[2.0,1.26,2.17,0.12,3.13,0.625,0.49];
TORSO_MAX_POSITION_LIMITS=[1.181,0.666,0.255];
RIGHT_ARM_MIN_POSITION_LIMITS=[-2.85,-1.26,-3.1,-0.12,-2.019,-0.625,-0.49];
LEFT_ARM_MIN_POSITION_LIMITS=[-2.85,-1.51,-3.09,-2.174,-2.018,-0.62,-0.36];
TORSO_MIN_POSITION_LIMITS=[-1.329,-0.13,-0.23];

RIGHT_LEG_MAX_POSITION_LIMITS=[0.41,0.467,1.619,2.057,0.875,0.348];
RIGHT_LEG_MIN_POSITION_LIMITS=[-1.1,-0.5515,-2.42,-0.083,-0.86,-0.349];


LEFT_LEG_MAX_POSITION_LIMITS=[0.348,0.875,2.057,1.619,0.5515,1.1];
LEFT_LEG_MIN_POSITION_LIMITS=[-0.349,-0.8644,-0.86,-2.42,-0.467,-0.4141];
NECK_MAX=[1.162,1.047,0.01];
NECK_MIN=[-0.02,-1.047,-0.872];

JOINT_NAMES=[LEFT_ARM;RIGHT_ARM;LEFT_LEG;RIGHT_LEG;TORSO;NECK];
JOINT_POSITION_MAX=[LEFT_ARM_MAX_POSITION_LIMITS';
    RIGHT_ARM_MAX_POSITION_LIMITS';
    LEFT_LEG_MAX_POSITION_LIMITS';
    RIGHT_LEG_MAX_POSITION_LIMITS';
    TORSO_MAX_POSITION_LIMITS';
    NECK_MAX'];

JOINT_POSITION_MIN=[LEFT_ARM_MIN_POSITION_LIMITS';
    RIGHT_ARM_MIN_POSITION_LIMITS';
    LEFT_LEG_MIN_POSITION_LIMITS';
    RIGHT_LEG_MIN_POSITION_LIMITS';
    TORSO_MIN_POSITION_LIMITS';
    NECK_MIN'];

LEFT_ARM_MAX_VELOCITY_LIMITS=[5.89;5.89;11.5;11.5;5;1;1]*0.5;
LEFT_ARM_MIN_VELOCITY_LIMITS=LEFT_ARM_MAX_VELOCITY_LIMITS*-1;

DANGER_VALUE=60;
a1=100;b1=10;

Tne=eye(4); Tne(2,4)=[0.08];


lf_leftSupport=[0.16, 0.1 0.
    0.16, -0.1 0
    -0.08, -0.1 0
    -0.08,  0.1 0];



RIGHT_ARM={    'rightShoulderPitch'
    'rightShoulderRoll'
    'rightShoulderYaw'
    'rightElbowPitch'
    'rightForearmYaw'
    'rightWristRoll'
    'rightWristPitch'};
LEFT_ARM={    'leftShoulderPitch'
    'leftShoulderRoll'
    'leftShoulderYaw'
    'leftElbowPitch'
    'leftForearmYaw'
    'leftWristRoll'
    'leftWristPitch'};
LEFT_ARM_LINKS={    'leftShoulderPitchLink'
    'leftShoulderRollLink'
    'leftShoulderYawLink'
    'leftElbowPitchLink'
    'leftForearmLink'
    'leftWristRollLink'
    'leftPalm'};

NECK={    'lowerNeckPitch'
    'neckYaw'
    'upperNeckPitch'};
TORSO={    'torsoYaw'
    'torsoPitch'
    'torsoRoll'};
RIGHT_LEG={'rightHipYaw'
    'rightHipRoll'
    'rightHipPitch'
    'rightKneePitch'
    'rightAnklePitch'
    'rightAnkleRoll'};
LEFT_LEG={'leftAnkleRoll'
    'leftAnklePitch'
    'leftKneePitch'
    'leftHipPitch'
    'leftHipRoll'
    'leftHipYaw'};


RobotChain.RIGHT_ARM_MAX_POSITION_LIMITS=RIGHT_ARM_MAX_POSITION_LIMITS;
RobotChain.LEFT_ARM_MAX_POSITION_LIMITS=LEFT_ARM_MAX_POSITION_LIMITS;
RobotChain.TORSO_MAX_POSITION_LIMITS=TORSO_MAX_POSITION_LIMITS;
RobotChain.RIGHT_ARM_MIN_POSITION_LIMITS=RIGHT_ARM_MIN_POSITION_LIMITS;
RobotChain.LEFT_ARM_MIN_POSITION_LIMITS=LEFT_ARM_MIN_POSITION_LIMITS;
RobotChain.TORSO_MIN_POSITION_LIMITS=TORSO_MIN_POSITION_LIMITS;

RobotChain.RIGHT_LEG_MAX_POSITION_LIMITS=RIGHT_LEG_MAX_POSITION_LIMITS;
RobotChain.RIGHT_LEG_MIN_POSITION_LIMITS=RIGHT_LEG_MIN_POSITION_LIMITS;
RobotChain.LEFT_LEG=LEFT_LEG;
RobotChain.RIGHT_LEG=RIGHT_LEG;
RobotChain.TORSO=TORSO;
RobotChain.LEFT_ARM=LEFT_ARM;
RobotChain.LEFT_ARM_LINKS=LEFT_ARM_LINKS;
RobotChain.RIGHT_ARM=RIGHT_ARM;

RobotChain.LEFT_LEG_MAX_POSITION_LIMITS=LEFT_LEG_MAX_POSITION_LIMITS;
RobotChain.LEFT_LEG_MIN_POSITION_LIMITS=LEFT_LEG_MIN_POSITION_LIMITS;
RobotChain.JOINT_NAMES=JOINT_NAMES;
RobotChain.JOINT_POSITION_MAX=JOINT_POSITION_MAX;
RobotChain.JOINT_POSITION_MIN=JOINT_POSITION_MIN;
RobotChain.LEFT_ARM_MAX_VELOCITY_LIMITS=LEFT_ARM_MAX_VELOCITY_LIMITS;
RobotChain.LEFT_ARM_MIN_VELOCITY_LIMITS=LEFT_ARM_MIN_VELOCITY_LIMITS;
RobotChain.Tne=Tne;
RobotChain.lf_leftSupport=lf_leftSupport;
gains.a1=a1;
gains.b1=b1;
gains.DANGER_VALUE=DANGER_VALUE;

end