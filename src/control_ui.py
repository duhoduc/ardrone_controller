#!/usr/bin/env python
import rospy
from ardrone_controller.msg import Goal, Waypoints
from std_msgs.msg import Empty, Float32, Float32MultiArray
from geometry_msgs.msg import Twist

height =  1.0
yaw_ref = 0.5
#goals = [[0, 0, height, 0.941 ],
#         [1.460, 0.90, height, 0.941 ],
#         [-1.54, -1.27, height, 0.933 ]
#        ]
goals = [[0, 0, height, 0],
         [1.5, 0, height, -0.3],
         [-1.5, 0, height, 0.3],
         [0, 1.5, height, -0.3],
         [0, -1.5, height, 0.3]
        ]

wp_ref = [[5, 1.5, 0, height, yaw_ref],
        [20, 0, 0, height, 0.0],
        [30, -1.5, 0, height, -yaw_ref],
        [40, 0, 0, height, 0.0],
        [50, 1.5, 0, height, yaw_ref],
        [60, 0, 0, height, 0.0],
        [70, -1.5, 0, height, -yaw_ref],
        [80, 0, 0, height, 0.0],
        [90, 1.5, 0, height, yaw_ref],
        [100, 0, 0, height, 0.0],
        [110, -1.5, 0, height, -yaw_ref],
        [120,0, 0, height, 0.0],
        [130, 1.5, 0, height, yaw_ref],
        [140, 0, 0, height, 0.0],
        [150, -1.5, 0, height, -yaw_ref],
        [160, 0, 0, height, 0.0]]

yaw_ref = 0.0
wp_ref_0 = [[5, 1.5, 0, height, yaw_ref],
        [20, 0, 0, height, 0.0],
        [30, -1.5, 0, height, -yaw_ref],
        [40, 0, 0, height, 0.0],
        [50, 1.5, 0, height, yaw_ref],
        [60, 0, 0, height, 0.0],
        [70, -1.5, 0, height, -yaw_ref],
        [80, 0, 0, height, 0.0],
        [90, 1.5, 0, height, yaw_ref],
        [100, 0, 0, height, 0.0],
        [110, -1.5, 0, height, -yaw_ref],
        [120,0, 0, height, 0.0],
        [130, 1.5, 0, height, yaw_ref],
        [140, 0, 0, height, 0.0],
        [150, -1.5, 0, height, -yaw_ref],
        [160, 0, 0, height, 0.0]]

# Need to read from text file the path, do it tomorrow

def talker():
    pubGoal = rospy.Publisher('arcontroller/goal', Goal, queue_size=10)
    pubWaypoints = rospy.Publisher('arcontroller/waypoints', Waypoints, queue_size=10)

    goal = Goal()
    wp_msg = Waypoints()
    rospy.loginfo(wp_msg)
    pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
    pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pubReset = rospy.Publisher('ardrone/reset', Empty, queue_size=1)

    with open('/home/duho46/catkin_ws/src/ardrone_controller/src/squarePath.txt') as f:
        paths = f.read().splitlines()
    f.close()
    path_list = [path.split(',') for path in paths]
    wp_square = [[float(x) for x in y] for y in path_list]

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        choice = raw_input('Goal: (1)Center, (2)Front, (3)back, (4)right, (5)Left, (6)yaw waypoints, (7)0 yaw waypoints, (8),to square, (9) square, (l): landing, (t) takeoff, (r) Reset \n')
        if choice == '1':
            goal.t = -1
            goal.x = goals[0][0]
            goal.y = goals[0][1]
            goal.z = goals[0][2]
            goal.yaw = goals[0][3]
            rospy.loginfo(goal)
            pubGoal.publish(goal)
        elif choice == '2':
            goal.t = -1
            goal.x = goals[1][0]
            goal.y = goals[1][1]
            goal.z = goals[1][2]
            goal.yaw = goals[1][3]
            pubGoal.publish(goal)
        elif choice == '3':
            goal.t = -1
            goal.x = goals[2][0]
            goal.y = goals[2][1]
            goal.z = goals[2][2]
            goal.yaw = goals[2][3]
            pubGoal.publish(goal)
        elif choice == '4':
            #pass#self.waypoint_follower(self.points_forward)
            goal.t = -1
            goal.x = goals[3][0]
            goal.y = goals[3][1]
            goal.z = goals[3][2]
            goal.yaw = goals[3][3]
            pubGoal.publish(goal)
        elif choice == '5':
            goal.t = -1
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
                goal.t = wp_ref[t][0]*0.5
                goal.x = wp_ref[t][1]
                goal.y = wp_ref[t][2]
                goal.z = wp_ref[t][3]
                goal.yaw = wp_ref[t][4]
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
                goal.t = wp_ref_0[t][0]*0.5
                goal.x = wp_ref_0[t][1]
                goal.y = wp_ref_0[t][2]
                goal.z = wp_ref_0[t][3]
                goal.yaw = wp_ref_0[t][4]
                wp_msg.waypoints.append(goal)
                #rospy.loginfo(goal)wp_msg
            wp_msg.len = len(wp_ref_0)
            #rospy.loginfo(wp_msg)
            pubWaypoints.publish(wp_msg)
        elif choice == '8':
            goal = Goal()
            goal.t = -1
            goal.x = wp_square[0][1]
            goal.y = wp_square[0][2]
            goal.z = wp_square[0][3]
            goal.yaw = wp_square[0][4]
            pubGoal.publish(goal)
        elif choice == '9':
            wp_msg = Waypoints()
            for wp in wp_square:
                goal = Goal()
                goal.t = wp[0]*0.5
                goal.x = wp[1]
                goal.y = wp[2]
                goal.z = wp[3]
                goal.yaw = wp[4]
                wp_msg.waypoints.append(goal)
            wp_msg.len = len(wp_square)
            pubWaypoints.publish(wp_msg)
        elif choice == 'l': # Landing
            msg = Twist()
            pubCmd.publish(msg)
            pubLand.publish()
        elif choice == 't':
            pubTakeoff.publish()
        elif choice == 'r':
            msg = Empty()
            pubReset.publish(msg)
        else:
            pass
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
