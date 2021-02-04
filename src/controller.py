#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from tf import TransformListener
from std_msgs.msg import Empty, Float32, Float32MultiArray
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import FlightAnim
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ardrone_controller.msg import Goal, Waypoints, ARDroneData

height =  1.0

class PID:
    def __init__(self, kp, kd, ki, minOutput, maxOutput, name):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.integral = 0.0
        self.anti_windup = 1
        self.previousError = 0.0
        self.previousTime = rospy.get_time()
        self.pubOutput = rospy.Publisher('pid/output/' + name, Float32, queue_size=1)
        self.pubError = rospy.Publisher('pid/error/' + name, Float32, queue_size=1)

    def update(self, value, targetValue):
        time = rospy.get_time()
        dt = time - self.previousTime
        error = targetValue - value
        self.integral += error * dt

        # We need anti windup here
        if self.integral < -self.anti_windup:
            self.integral = -self.anti_windup
        elif self.integral > self.anti_windup:
            self.integral = self.anti_windup

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
    Flying2 = 7 # gotohover
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
        self.lastImu = None
        self.lastMag = None
        self.current_pose = None
        self.current_odom = None
        self.lastState = State.Unknown
        self.command = Twist()
        self.cmd_freq = 1.0/400.0

        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber("ardrone/navdata", Navdata, self.on_navdata)
        rospy.Subscriber("ardrone/imu", Imu, self.on_imu)
        rospy.Subscriber("ardrone/mag", Vector3Stamped, self.on_mag)
        rospy.Subscriber("arcontroller/goal", Goal , self.on_goal)
        rospy.Subscriber("arcontroller/waypoints", Waypoints, self.on_waypoints)
        rospy.Subscriber("qualisys/ARDrone/pose",PoseStamped,self.get_current_pose)
        rospy.Subscriber("qualisys/ARDrone/odom",Odometry,self.get_current_odom)
        self.pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pubDroneData = rospy.Publisher('droneData', ARDroneData, queue_size=1)
        self.setFlightAnimation = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim)

        self.commandTimer = rospy.Timer(rospy.Duration(self.cmd_freq),self.sendCommand)

        self.listener = TransformListener()
        self.action = Controller.ActionTakeOff
        self.previousDebugTime = rospy.get_time()
		

        self.pidX = PID(0.5, 0.12, 0.0, -1, 1, "x")
        self.pidY = PID(0.5, 0.12, 0.0, -1, 1, "y")
        self.pidZ = PID(1.0, 0.1, 0.25, -1.0, 1.0, "z")
        self.pidYaw = PID(0.5, 0.15, 0.0, -0.6, 0.6, "yaw")
        self.scale = 1.0
        

        self.goal = [-1,0,0,height,0] #set it to center to start
        self.goal_done = False
        self.waypoints = None

    def get_current_pose(self,data):
        self.current_pose = data

    def get_current_odom(self,data):
        self.current_odom = data

    def on_imu(self, data):
        self.lastImu = data

    def on_mag(self, data):
        self.lastMag = data

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
            self.pubCmd.publish(msg)
        rospy.sleep(1)

    def on_goal(self,data):
        rospy.loginfo('New goal.')
        self.goal = [data.t, data.x,data.y,data.z,data.yaw]
        self.goal_done = False

    def sendCommand(self)
        self.pubCmd.publish(self.command)

    def on_waypoints(self,data):
        rospy.loginfo('New waypoints.')
        self.waypoints = []
        for d in range(data.len):
            self.waypoints.append([data.waypoints[d].t, data.waypoints[d].x, data.waypoints[d].y, data.waypoints[d].z, data.waypoints[d].yaw])
        rospy.loginfo(self.waypoints)

   
    def waypoint_follower(self, points): 
        index = 0
        rospy.loginfo(points)
        self.goal = points[index] #get the first point
        time_wp = [goal[0] for goal in points]
        minX = .05
        minY = .05 
        time0_wp = rospy.get_time()
        while True:#for i in range(0,points.len()):
            #goal = points[i]
            # transform target world coordinates into local coordinates
            targetWorld = PoseStamped()
            t = self.listener.getLatestCommonTime("/ARDrone", "/mocap")
            if self.listener.canTransform("/ARDrone", "/mocap", t):
                # Get starting time
                startCmdTime = rospy.get_time()

                targetWorld.header.stamp = t
                targetWorld.header.frame_id = "mocap"
                targetWorld.pose.position.x = self.goal[1]
                targetWorld.pose.position.y = self.goal[2]
                targetWorld.pose.position.z = self.goal[3]
                quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goal[4])
                
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
                msg.linear.x = self.scale*self.pidX.update(0.0, targetDrone.pose.position.x)
                msg.linear.y = self.scale*self.pidY.update(0.0, targetDrone.pose.position.y)
                msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                msg.angular.z = self.pidYaw.update(0.0, euler[2])
                # disable hover mode
                msg.angular.x = 0
                self.command = msg
                #self.pubCmd.publish(msg)


                # WE define the drone data message and publish
                drone_msg = ARDroneData()
                
                drone_msg.header.stamp = t
                drone_msg.header.frame_id = 'drone_data'
                drone_msg.navdata = self.lastNavdata
                drone_msg.imu = self.lastImu
                drone_msg.mag = self.lastMag
                drone_msg.pose = self.current_pose
                drone_msg.odom = self.current_odom
                drone_msg.cmd = msg
                self.pubDroneData.publish(drone_msg)

                error_xy = math.sqrt(targetDrone.pose.position.x**2+targetDrone.pose.position.y**2)
                current_time_wp = rospy.get_time()
                if self.goal[0] < 0: #-1 implying that waypoints is not time-dependent
                    # goal t, x, y, z, yaw
                    #self.goal = points[index]
                    if (error_xy < 0.2
                    and math.fabs(targetDrone.pose.position.z) < 0.2
                    and math.fabs(euler[2]) < math.radians(5)):
                        
                        if (index < len(points)-1):
                            index += 1                             
                            self.goal = points[index]
                        else:
                            return
                else:
                    diff_time_wp = current_time_wp-time0_wp
                    if diff_time_wp<=time_wp[-1]:
                        index = next(x for x, val in enumerate(time_wp) if val >= diff_time_wp)
                        self.goal = points[index]
                    else:
                        return

                #time = rospy.get_time()
                diff_time_log = current_time_wp-self.previousDebugTime
                if diff_time_log > 0.5:
                #log_msg = "Current pos:" + [targetDrone.pose.position.x, targetDrone.pose.position.y,targetDrone.pose.position.z]
                    rospy.loginfo('--------------------------------------')
                    rospy.loginfo('Control:%.2f,%.2f,%.2f,%.2f | bat: %.2f',msg.linear.x, msg.linear.y,msg.linear.z,msg.angular.z,self.lastNavdata.batteryPercent)
                    rospy.loginfo('Current position: [%.2f, %.2f, %.2f, %.2f, %.2f]',diff_time_wp, self.current_pose.pose.position.x,self.current_pose.pose.position.y,self.current_pose.pose.position.z,euler[2])
                    rospy.loginfo('Current goal:%.2f,%.2f,%.2f,%.2f,%.2f',self.goal[0], self.goal[1], self.goal[2], self.goal[3], self.goal[4])
                    rospy.loginfo('Error: %.2f,%.2f,%.2f', error_xy, math.fabs(targetDrone.pose.position.z), math.fabs(euler[2]))
                    rospy.loginfo('--------------------------------------')
                    self.previousDebugTime = current_time_wp

                # if (math.fabs(targetDrone.pose.position.x) < 0.1
                #     and math.fabs(targetDrone.pose.position.y) < 0.1
                #     and math.fabs(targetDrone.pose.position.z) < 0.2
                #     and math.fabs(euler[2]) < math.radians(10)):
                        
                #     if (index < len(points)-1):
                #         index += 1                             
                #         self.goal = points[index]
                #     else:
                #         return

                # Check time duration and sleep
                # durationCmd = self.cmd_freq-(rospy.get_time() - startCmdTime)
                # if durationCmd>0: # Meaning the execution < cmd_freq
                #     rospy.sleep(durationCmd)
                        


    def go_to_goal(self, goal):
        #rospy.loginfo('Going to goal')
        #rospy.loginfo(goal)
        self.goal = goal
        # transform target world coordinates into local coordinates
        targetWorld = PoseStamped()


        t = self.listener.getLatestCommonTime("/ARDrone", "/mocap")
        if self.listener.canTransform("/ARDrone", "/mocap", t):
            # Get starting time
            startCmdTime = rospy.get_time()

            targetWorld.header.stamp = t
            targetWorld.header.frame_id = "mocap"
            targetWorld.pose.position.x = goal[1]
            targetWorld.pose.position.y = goal[2]
            targetWorld.pose.position.z = goal[3]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, goal[4])  
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
            msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
            msg.angular.z = self.pidYaw.update(0.0, euler[2])
            # disable hover mode
            msg.angular.x = 0
            #self.pubCmd.publish(msg)
            self.command = msg

            # WE define the drone data message and publish
            drone_msg = ARDroneData()
            
            drone_msg.header.stamp = rospy.get_rostime()
            drone_msg.header.frame_id = 'drone_data'
            drone_msg.navdata = self.lastNavdata
            drone_msg.imu = self.lastImu
            drone_msg.mag = self.lastMag
            drone_msg.pose = self.current_pose
            drone_msg.odom = self.current_odom
            drone_msg.cmd = msg
            self.pubDroneData.publish(drone_msg)

            error_xy = math.sqrt(targetDrone.pose.position.x**2+targetDrone.pose.position.y**2)
            time = rospy.get_time()
            if time-self.previousDebugTime>1:
                #log_msg = "Current pos:" + [targetDrone.pose.position.x, targetDrone.pose.position.y,targetDrone.pose.position.z]
                rospy.loginfo('--------------------------------------')
                rospy.loginfo('Control:%.2f,%.2f,%.2f,%.2f,bat:%.2f',msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z,self.lastNavdata.batteryPercent)
                rospy.loginfo('Current goal:%.2f,%.2f,%.2f,%.2f',goal[1], goal[2],goal[3],goal[4])
                rospy.loginfo('Current pose:%.2f,%.2f,%.2f',self.current_pose.pose.position.x, self.current_pose.pose.position.y,self.current_pose.pose.position.z)
                rospy.loginfo('Error: %.2f,%.2f,%.2f', error_xy, math.fabs(targetDrone.pose.position.z), math.fabs(euler[2]))
                self.previousDebugTime=time
                if self.goal_done:
                    rospy.loginfo("Goal done.")
                rospy.loginfo('-------------------------------------')
            
            if (error_xy < 0.2 and math.fabs(targetDrone.pose.position.z) < 0.2 and math.fabs(euler[2]) < math.radians(5)):
                self.goal_done = True
            else:
            	self.goal_done = False
                #time = rospy.get_time()
                #if time-self.previousDebugTime>1:
                #	rospy.loginfo("Goal done.")
            # Check time duration and sleep
            # durationCmd = self.cmd_freq-(rospy.get_time() - startCmdTime)
            # if durationCmd>0: # Meaning the execution < cmd_freq
            #     rospy.sleep(durationCmd)


    def run(self):
        while not rospy.is_shutdown():
            if self.action == Controller.ActionTakeOff:
                if self.lastState == State.Landed:
                    #rospy.loginfo('Taking off.')
                    #self.pubTakeoff.publish()
                elif self.lastState == State.Hovering or self.lastState == State.Flying or self.lastState == State.Flying2:
                    self.action = Controller.ActionHover
            elif self.action == Controller.ActionLand:
                msg = Twist()
                self.pubCmd.publish(msg)
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
