#!/usr/bin/env python
import rospy
from ardrone_controller.msg import Goal, Waypoints
from std_msgs.msg import Empty, Float32, Float32MultiArray

height =  1.0
yaw_ref = 0.5
#goals = [[0, 0, height, 0.941 ],
#         [1.460, 0.90, height, 0.941 ],
#         [-1.54, -1.27, height, 0.933 ]
#        ]
goals = [[0, 0, height, 0],
         [1.5, 0, height, 0.3],
         [-1.5, 0, height, 0.3],
         [1.5, 0, height, -0.3],
         [-1.5, 0, height, -0.3]
        ]

wp_ref = [[1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0]]

yaw_ref = 0.0
wp_ref_0 = [[1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0],
        [1.5, 0, height, yaw_ref],
        [0, 0, height, 0.0],
        [-1.5, 0, height, -yaw_ref],
        [0, 0, height, 0.0]]

def talker():
    pubGoal = rospy.Publisher('arcontroller/goal', Goal, queue_size=10)
    pubWaypoints = rospy.Publisher('arcontroller/waypoints', Waypoints, queue_size=10)

    goal = Goal()
    wp_msg = Waypoints()
    rospy.loginfo(wp_msg)
    pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        choice = raw_input('Choose a goal: (1)Center, (2)Back1, (3)Front1, (4)Back2, (5)Front2, (6)yaw waypoints, (7)0 yaw waypoints \n')
        if choice == '1':
            goal.x = goals[0][0]
            goal.y = goals[0][1]
            goal.z = goals[0][2]
            goal.yaw = goals[0][3]
            rospy.loginfo(goal)
            pubGoal.publish(goal)
        elif choice == '2':
            goal.x = goals[1][0]
            goal.y = goals[1][1]
            goal.z = goals[1][2]
            goal.yaw = goals[1][3]
            pubGoal.publish(goal)
        elif choice == '3':
            goal.x = goals[2][0]
            goal.y = goals[2][1]
            goal.z = goals[2][2]
            goal.yaw = goals[2][3]
            pubGoal.publish(goal)
        elif choice == '4':
            #pass#self.waypoint_follower(self.points_forward)
            goal.x = goals[3][0]
            goal.y = goals[3][1]
            goal.z = goals[3][2]
            goal.yaw = goals[3][3]
            pubGoal.publish(goal)
        elif choice == '5':
            goal.x = goals[4][0]
            goal.y = goals[4][1]
            goal.z = goals[4][2]
            goal.yaw = goals[4][3]
            pubGoal.publish(goal)
        elif choice == '6':
            wp_msg = Waypoints()
            #rospy.loginfo(wp_msg)
            #rospy.loginfo(type(wp_msg.waypoints))
            for t in range(len(wp_ref)):
                goal = Goal()
                goal.x = wp_ref[t][0]
                goal.y = wp_ref[t][1]
                goal.z = wp_ref[t][2]
                goal.yaw = wp_ref[t][3]
                wp_msg.waypoints.append(goal)
                #rospy.loginfo(goal)wp_msg
            wp_msg.len = len(wp_ref)
            #rospy.loginfo(wp_msg)
            pubWaypoints.publish(wp_msg)
        elif choice == '7':
            wp_msg = Waypoints()
            #rospy.loginfo(wp_msg)
            #rospy.loginfo(type(wp_msg.waypoints))
            for t in range(len(wp_ref_0)):
                goal = Goal()
                goal.x = wp_ref_0[t][0]
                goal.y = wp_ref_0[t][1]
                goal.z = wp_ref_0[t][2]
                goal.yaw = wp_ref_0[t][3]
                wp_msg.waypoints.append(goal)
                #rospy.loginfo(goal)wp_msg
            wp_msg.len = len(wp_ref)
            #rospy.loginfo(wp_msg)
            pubWaypoints.publish(wp_msg)
        elif choice == '8': # Landing
            msg = Twist()
            pubCmd.publish(msg)
            pubLand.publish()
        else:
            pass
        rospy.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
