#!/usr/bin/env python

# A simple dummy joint state publisher that resets the robot's state to the home state
#
#
#
#

import roslib
import rospy
import sensor_msgs.msg
import geometry_msgs.msg





if __name__ == '__main__':
    rospy.init_node('dummy_joint_publisher')
    joint_timeout=rospy.get_param('~reset_time',5.0)

    pub = rospy.Publisher('dummy_joint_states', sensor_msgs.msg.JointState, queue_size=10)
    joint_states=sensor_msgs.msg.JointState()
    joint_states.header.frame_id='/world'
    joint_states.name=['leftHipYaw', 'leftHipRoll', 'leftHipPitch', 'leftKneePitch',  'leftAnklePitch', 'leftAnkleRoll',
    'rightHipYaw', 'rightHipRoll', 'rightHipPitch', 'rightKneePitch', 'rightAnklePitch', 'rightAnkleRoll',
    'torsoYaw', 'torsoPitch', 'torsoRoll',
    'leftShoulderPitch', 'leftShoulderRoll', 'leftShoulderYaw', 'leftElbowPitch', 'leftForearmYaw', 'leftWristRoll', 'leftWristPitch',
    'lowerNeckPitch', 'neckYaw', 'upperNeckPitch',
    'hokuyo_joint',
    'rightShoulderPitch', 'rightShoulderRoll', 'rightShoulderYaw', 'rightElbowPitch', 'rightForearmYaw', 'rightWristRoll', 'rightWristPitch']
    joint_states.position=[-0.006612501107156277, -0.06531478464603424, -0.6120110154151917, 1.3423011302947998, -0.7300236821174622,
                           0.06532349437475204, 0.00880342535674572, 0.06476577371358871, -0.6120499968528748, 1.3423198461532593, -0.730003833770752,
                           -0.06475304067134857, -1.4374360944202635e-05, 0.00025919225299730897, 8.833262654661667e-06, -0.19992607831954956,
                           -1.2000226974487305, 0.7000040411949158, -1.4999690055847168, 1.3000966310501099, -0.0003523259947542101, -5.6606884754728526e-05,
                           -1.870074470389227e-06, 1.6165125771294697e-06, 1.5674970200052485e-05, 0.0, -0.19992856681346893, 1.2000117301940918,
                           0.7000198364257812, 1.4999674558639526, 1.3003301620483398, 0.00025184423429891467, -4.0509541577193886e-05]

    joint_states.velocity=[-1.3594757547252811e-05, -9.93789653875865e-05, 0.0010733174858614802, 2.3993146896827966e-05, -0.0052507356740534306,
                           5.268329550744966e-05, -9.805859917833004e-06, -6.837357796030119e-05, 0.0010551272425800562, 2.7946285626967438e-05,
                           -0.0051505593582987785, -1.4029743624632829e-06, -2.501600465620868e-05, 4.607912342180498e-05, -8.973347576102242e-06,
                           7.308176282094792e-05, -1.4843751159787644e-05, -0.00015224801609292626, -0.00014270645624492317, -0.005009092856198549,
                           -0.0025287503376603127, -0.0017269192030653358, 0.0008780692587606609, -6.56939300824888e-05, -0.004387030843645334, 0.0,
                           0.0, 0.0, 0.00010672109056031331, -0.00015436881221830845, -0.0, -0.0008916097576729953, 0.0]

    joint_states.effort=[0.0019778998102992773, 14.825940132141113, 4.512240886688232, -126.27506256103516, 43.978172302246094, 0.10594018548727036,
                         0.0008360818028450012, -14.078537940979004, 4.456239700317383, -125.96624755859375, 43.879974365234375, 0.10768258571624756,
                         -0.004108868073672056, 24.568035125732422, -0.3226260542869568, -10.699654579162598, 4.587111473083496, -3.2294766902923584,
                         -6.132748126983643, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -11.247946739196777, -3.729247570037842, -3.1378769874572754,
                         6.132091045379639, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():
        joint_states.header.seq=joint_states.header.seq+1
        joint_states.header.stamp=rospy.Time.now()
        pub.publish(joint_states)
        rospy.sleep(joint_timeout)
    rospy.spin()
