#!/usr/bin/env python
import rospy
from ardrone_controller.msg import Goal, Waypoints


height =  1.25
#goals = [[0, 0, height, 0.941 ],
#         [1.460, 0.90, height, 0.941 ],
#         [-1.54, -1.27, height, 0.933 ]
#        ]
goals = [[0, 0, height, 0 ],
         [1.5, 0, height, 0 ],
         [-1,5, 0, height, 0]
        ]

waypoints = [[1.5, 0, height, 0],
        [-1.5, 0, height, 0],
        [1.5, 0, height, 0],
        [-1.5, 0, height, 0],
        [1.5, 0, height, 0],
        [-1.5, 0, height, 0],
        [1.5, 0, height, 0],
        [-1.5, 0, height, 0],
        [0, 0, height, 0]]

def talker():
    pubGoal = rospy.Publisher('arcontroller/goal', Goal, queue_size=10)
    pubWaypoints = rospy.Publisher('arcontroller/waypoints', Waypoints, queue_size=10)

    goal = Goal()
    waypoints = Waypoints()
    pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        choice = raw_input('Choose a goal: (1)Center, (2)Back, (3)Front, (4)Forward Waypoints, (5)Land, \n')
        if choice == '1':
            goal.x = goals[0][0]
            goal.y = goals[0][1]
            goal.z = goals[0][2]
            goal.yaw = goals[0][3]
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
            pubWaypoints.publish(waypoints)
        elif choice == 5:
            pubLand.publish()
        else:
            pass
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass