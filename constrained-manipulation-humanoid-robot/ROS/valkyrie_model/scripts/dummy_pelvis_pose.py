#!/usr/bin/env python

# A simple pelvis state broadcaster. The topic is required by a optimization process
# 
#
#
#


import roslib
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import tf





if __name__ == '__main__':
    rospy.init_node('dummy_pelvis_pose')
    pub = rospy.Publisher('/ihmc_ros/valkyrie/output/robot_pose', nav_msgs.msg.Odometry, queue_size=10)

    listener = tf.TransformListener()
    pelvis_pose=nav_msgs.msg.Odometry()
    pelvis_pose.header.frame_id='world'
    pelvis_pose.child_frame_id='pelvis'
    pelvis_pose.pose.covariance=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pelvis_pose.twist.twist.linear.x=0.0
    pelvis_pose.twist.twist.linear.y=0.0
    pelvis_pose.twist.twist.linear.z=0.0
    pelvis_pose.twist.twist.angular.x=0.0
    pelvis_pose.twist.twist.angular.y=0.0
    pelvis_pose.twist.twist.angular.z=0.0


    r = rospy.Rate(200) # 10hz                
    while not rospy.is_shutdown():
       
        try:
            (trans,rot) = listener.lookupTransform('/world', '/pelvis', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        pelvis_pose.pose.pose.position.x=trans[0]
        pelvis_pose.pose.pose.position.y=trans[1]
        pelvis_pose.pose.pose.position.z=trans[2]
        pelvis_pose.pose.pose.orientation.x=rot[0]
        pelvis_pose.pose.pose.orientation.y=rot[1]
        pelvis_pose.pose.pose.orientation.z=rot[2]
        pelvis_pose.pose.pose.orientation.w=rot[3]
        pelvis_pose.header.seq=pelvis_pose.header.seq+1
        pelvis_pose.header.stamp=rospy.Time.now()
        pub.publish(pelvis_pose)
        r.sleep()

