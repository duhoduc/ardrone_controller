#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from tf import TransformListener
from std_msgs.msg import Empty, Float32, Float32MultiArray
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import FlightAnim
from geometry_msgs.msg import Twist, PoseStamped
from ardrone_controller.msg import Goal, Waypoints

height =  1.0

class PID:
    def __init__(self, kp, kd, ki, minOutput, maxOutput, name):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.previousError = 0.0
        self.previousTime = rospy.get_time()
        self.pubOutput = rospy.Publisher('pid/output/' + name, Float32, queue_size=1)
        self.pubError = rospy.Publisher('pid/error/' + name, Float32, queue_size=1)

    def update(self, value, targetValue):
        time = rospy.get_time()
        dt = time - self.previousTime
        error = targetValue - value
        self.integral += error * dt
        output = self.kp * error + self.kd * (error - self.previousError) / dt + self.ki * self.integral
        self.previousError = error
        self.previousTime = time
        self.pubOutput.publish(output)
        self.pubError.publish(error)
        return max(min(output, self.maxOutput), self.minOutput)

class State():
    Unknown = 0
    Inited = 1
    Landed = 2
    Flying = 3
    Flying2 = 7
    Hovering = 4
    Test = 5
    TakingOff = 6
    Landing = 8
    Looping = 9

class Controller():
    ActionTakeOff = 0
    ActionHover = 1
    ActionLand = 2
    ActionAnimation = 3
    

    def __init__(self):
        self.lastNavdata = None
        self.current_pose = None
        self.lastState = State.Unknown
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber("ardrone/navdata", Navdata, self.on_navdata)
        rospy.Subscriber("arcontroller/goal", Goal , self.on_goal)
        rospy.Subscriber("arcontroller/waypoints", Waypoints, self.on_waypoints)
        rospy.Subscriber("qualisys/ARDrone",PoseStamped,self.get_current_pose)
        self.pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.setFlightAnimation = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim)

        self.listener = TransformListener()
        self.action = Controller.ActionTakeOff
        self.previousTime = rospy.get_time()
		

        self.pidX = PID(0.5, 0.12, 0.0, -1, 1, "x")
        self.pidY = PID(0.5, 0.12, 0.0, -1, 1, "y")
        self.pidZ = PID(1.0, 0.1, 0.25, -1.0, 1.0, "z")
        self.pidYaw = PID(0.5, 0.15, 0.0, -0.6, 0.6, "yaw")
        self.scale = 0.5

        # X, Y, Z, Yaw
        #self.goals = [
        #        [0, 0, height, 0.941 ],
        #        [1.789, 1.158, height, 0.941 ],
        #        [-2.035, -1.539, height, 0.933 ]
        #   ]

        #self.points_forward = [self.goals[1],self.goals[0],self.goals[2]]
        #[[0, 0, height, 0.941 ],
                #[-2.035, -1.539, height, 0.933 ]]

        #self.goalIndex = 0
        self.goal = [0,0,height,0] #set it to center to start
        self.goal_done = False
        self.waypoints = None

    def get_current_pose(self,data):
        self.current_pose = data

    def on_navdata(self, data):
        self.lastNavdata = data
        if data.state != self.lastState:
            rospy.loginfo("State Changed: " + str(data.state))
            self.lastState = data.state

    def on_shutdown(self):
        rospy.loginfo("Shutdown: try to land...")
        msg = Twist()
        for i in range(0, 1000):
            self.pubLand.publish()
            self.pubNav.publish(msg)
        rospy.sleep(1)

    def on_goal(self,data):
        rospy.loginfo('New goal.')
        self.goal = [data.x,data.y,data.z,data.yaw]
        self.goal_done = False

    def on_waypoints(self,data):
        rospy.loginfo('New waypoints.')
        self.waypoints = []
        for d in range(0,data.len):
            self.waypoints.append([data.waypoints[d].x, data.waypoints[d].y, data.waypoints[d].z, data.waypoints[d].yaw])
        rospy.loginfo(self.waypoints)

   
    def waypoint_follower(self, points): 
        index = 0
        rospy.loginfo(points)
        self.goal = points[index] #get the first point
        minX = .05
        minY = .05 

        while True:#for i in range(0,points.len()):
            #goal = points[i]
            # transform target world coordinates into local coordinates
            targetWorld = PoseStamped()
            t = self.listener.getLatestCommonTime("/ARDrone", "/mocap")
            if self.listener.canTransform("/ARDrone", "/mocap", t):
                targetWorld.header.stamp = t
                targetWorld.header.frame_id = "mocap"
                targetWorld.pose.position.x = self.goal[0]
                targetWorld.pose.position.y = self.goal[1]
                targetWorld.pose.position.z = self.goal[2]
                quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goal[3])
                targetWorld.pose.orientation.x = quaternion[0]
                targetWorld.pose.orientation.y = quaternion[1]
                targetWorld.pose.orientation.z = quaternion[2]
                targetWorld.pose.orientation.w = quaternion[3]

                targetDrone = self.listener.transformPose("/ARDrone", targetWorld)

                quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)

                # Run PID controller and send navigation message
                msg = Twist()
                scale = 0.4
                msg.linear.x = self.scale*self.pidX.update(0.0, targetDrone.pose.position.x)
                msg.linear.y = self.scale*self.pidY.update(0.0, targetDrone.pose.position.y)
                    

                #pid_x = scale*self.pidX.update(0.0, targetDrone.pose.position.x)
                #pid_y = scale*self.pidY.update(0.0, targetDrone.pose.position.y)
                #true_yaw = goal[3]-euler[2]
                #msg.linear.x = math.cos(true_yaw)*pid_x + math.sin(true_yaw)*pid_y
                #msg.linear.y = -math.sin(true_yaw)*pid_x + math.cos(true_yaw)*pid_y

                # Do not stop when pass a waypoint
                if (index != len(points)-1):
                    if (math.fabs(msg.linear.x) < minX) :
                        if (msg.linear.x < 0) :
                            msg.linear.x = -minX
                        elif (msg.linear.x > 0):
                            msg.linear.x = minX
                    if (math.fabs(msg.linear.y) < minY) :
                        if (msg.linear.y < 0) :
                            msg.linear.y = -minY
                        elif (msg.linear.y > 0):
                            msg.linear.y = minY

                msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                msg.angular.z = self.pidYaw.update(0.0, euler[2])
                # disable hover mode
                msg.angular.x = 0
                self.pubNav.publish(msg)

                time = rospy.get_time()
                if time-self.previousTime>1:
                #log_msg = "Current pos:" + [targetDrone.pose.position.x, targetDrone.pose.position.y,targetDrone.pose.position.z]
                    rospy.loginfo('Current pos: x:%.2f, y:%.2f, z:%.2f, yaw:%.2f, bat:%.2f',targetDrone.pose.position.x,targetDrone.pose.position.y,targetDrone.pose.position.z,euler[2],self.lastNavdata.batteryPercent)
                    rospy.loginfo('COntrol:%.2f,%.2f,%.2f,%.2f',msg.linear.x, msg.linear.y,msg.linear.z,msg.angular.z)
                    self.previousTime =time

                if (math.fabs(targetDrone.pose.position.x) < 0.1
                    and math.fabs(targetDrone.pose.position.y) < 0.1
                    and math.fabs(targetDrone.pose.position.z) < 0.2
                    and math.fabs(euler[2]) < math.radians(10)):
                        
                    if (index < len(points)-1):
                        index += 1                             
                        self.goal = points[index] 
                    else:
                        return
                        


    def go_to_goal(self, goal):
        #rospy.loginfo('Going to goal')
        #rospy.loginfo(goal)
        self.goal = goal
        # transform target world coordinates into local coordinates
        targetWorld = PoseStamped()
        t = self.listener.getLatestCommonTime("/ARDrone", "/mocap")
        if self.listener.canTransform("/ARDrone", "/mocap", t):
            targetWorld.header.stamp = t
            targetWorld.header.frame_id = "mocap"
            targetWorld.pose.position.x = goal[0]
            targetWorld.pose.position.y = goal[1]
            targetWorld.pose.position.z = goal[2]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, goal[3])  
            targetWorld.pose.orientation.x = quaternion[0]
            targetWorld.pose.orientation.y = quaternion[1]
            targetWorld.pose.orientation.z = quaternion[2]
            targetWorld.pose.orientation.w = quaternion[3]

            targetDrone = self.listener.transformPose("/ARDrone", targetWorld)

            quaternion = (
                targetDrone.pose.orientation.x,
                targetDrone.pose.orientation.y,
                targetDrone.pose.orientation.z,
                targetDrone.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            # Run PID controller and send navigation message
            msg = Twist()
            scale = 0.4
            msg.linear.x = self.scale*self.pidX.update(0.0, targetDrone.pose.position.x)
            msg.linear.y = self.scale*self.pidY.update(0.0, targetDrone.pose.position.y)

            #pid_x = scale*self.pidX.update(0.0, targetDrone.pose.position.x)
            #pid_y = scale*self.pidY.update(0.0, targetDrone.pose.position.y)
            #true_yaw = goal[3]-euler[2]
            #msg.linear.x = math.cos(true_yaw)*pid_x + math.sin(true_yaw)*pid_y
            #msg.linear.y = -math.sin(true_yaw)*pid_x + math.cos(true_yaw)*pid_y

            msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
            msg.angular.z = self.pidYaw.update(0.0, euler[2])
            # disable hover mode
            msg.angular.x = 0
            self.pubNav.publish(msg)

            time = rospy.get_time()
            if time-self.previousTime>1:
                #log_msg = "Current pos:" + [targetDrone.pose.position.x, targetDrone.pose.position.y,targetDrone.pose.position.z]
                rospy.loginfo('Current pos: x:%.2f, y:%.2f, z:%.2f, yaw:%.2f, bat:%.2f',targetDrone.pose.position.x, targetDrone.pose.position.y,targetDrone.pose.position.z,euler[2],self.lastNavdata.batteryPercent)
                rospy.loginfo('COntrol:%.2f,%.2f,%.2f,%.2f',msg.linear.x, msg.linear.y,msg.linear.z,msg.angular.z)
                self.previousTime =time
                if self.goal_done:
                    rospy.loginfo("Goal done.")



            if (math.fabs(targetDrone.pose.position.x) < 0.1
                and math.fabs(targetDrone.pose.position.y) < 0.1
                and math.fabs(targetDrone.pose.position.z) < 0.2
                and math.fabs(euler[2]) < math.radians(5)):
                self.goal_done = True
                #rospy.loginfo("Goal done.") 
                #time = rospy.get_time()
                #if time-self.previousTime>1:
                #	rospy.loginfo("Goal done.")           


    def run(self):
        while not rospy.is_shutdown():
            if self.action == Controller.ActionTakeOff:
                if self.lastState == State.Landed:
                    #rospy.loginfo('Taking off.')
                    self.pubTakeoff.publish()
                elif self.lastState == State.Hovering or self.lastState == State.Flying or self.lastState == State.Flying2:
                    self.action = Controller.ActionHover
            elif self.action == Controller.ActionLand:
                msg = Twist()
                self.pubNav.publish(msg)
                self.pubLand.publish()
            elif self.action == Controller.ActionHover:
                if self.waypoints == None:
                    #if self.goal_done == False:
                    self.go_to_goal(self.goal)
                else:
                    self.waypoint_follower(self.waypoints)
                    self.waypoints = None

            rospy.sleep(0.01)


if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    controller.run()
