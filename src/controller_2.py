#!/usr/bin/env python

import rospy
import math
import tf
from tf import TransformListener
from std_msgs.msg import Empty, Float32
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import FlightAnim
from geometry_msgs.msg import Twist, PoseStamped

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
        self.lastState = State.Unknown
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber("ardrone/navdata", Navdata, self.on_navdata)
        self.pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.pubLand = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.setFlightAnimation = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim)

        self.listener = TransformListener()
        self.action = Controller.ActionTakeOff

        self.pidX = PID(0.2, 0.12, 0.0, -0.3, 0.3, "x")
        self.pidY = PID(0.2, 0.12, 0.0, -0.3, 0.3, "y")
        self.pidZ = PID(1.0, 0, 0.0, -1.0, 1.0, "z")
        self.pidYaw = PID(0.5, 0, 0.0, -0.6, 0.6, "yaw")

        # X, Y, Z, Yaw
        self.goals = [
                [0.0, 0.0, 2.0, math.radians(0) ],
                "ANIM",
                [0.0, 0.0, 2.0, math.radians(0) ],
                [1.0, 0.0, 1.5, math.radians(90)],
                [1.0, 1.0, 0.8, math.radians(180)],
                [-1.0, 1.0, 1.2, math.radians(0)],
                [1.0, 0.0, 0.8, math.radians(0)],
            ]
        self.goalIndex = 0

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

    def run(self):
        while not rospy.is_shutdown():
            if self.action == Controller.ActionTakeOff:
                if self.lastState == State.Landed:
                    self.pubTakeoff.publish()
                elif self.lastState == State.Hovering or self.lastState == State.Flying or self.lastState == State.Flying2:
                    self.action = Controller.ActionHover
            elif self.action == Controller.ActionLand:
                msg = Twist()
                self.pubCmd.publish(msg)
                self.pubLand.publish()
            elif self.action == Controller.ActionHover:
                rospy.loginfo('pid running')
                # transform target world coordinates into local coordinates
                targetWorld = PoseStamped()
                t = self.listener.getLatestCommonTime("/vicon/ar_drone/ar_drone", "/world")
                if self.listener.canTransform("/vicon/ar_drone/ar_drone", "/world", t):
                    targetWorld.header.stamp = t
                    targetWorld.header.frame_id = "world"
                    targetWorld.pose.position.x = self.goals[self.goalIndex][0]
                    targetWorld.pose.position.y = self.goals[self.goalIndex][1]
                    targetWorld.pose.position.z = self.goals[self.goalIndex][2]
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
                    targetWorld.pose.orientation.x = quaternion[0]
                    targetWorld.pose.orientation.y = quaternion[1]
                    targetWorld.pose.orientation.z = quaternion[2]
                    targetWorld.pose.orientation.w = quaternion[3]

                    targetDrone = self.listener.transformPose("/vicon/ar_drone/ar_drone", targetWorld)

                    quaternion = (
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)

                    # Run PID controller and send navigation message
                    msg = Twist()
                    #msg.linear.x = self.pidX.update(velX, targetVelX)
                    msg.linear.x = self.pidX.update(0.0, targetDrone.pose.position.x)
                    msg.linear.y = self.pidY.update(0.0, targetDrone.pose.position.y)
                    msg.linear.z = self.pidZ.update(0.0, targetDrone.pose.position.z)
                    msg.angular.z = self.pidYaw.update(0.0, euler[2])
                    # disable hover mode
                    msg.angular.x = 1
                    self.pubCmd.publish(msg)

                    if (math.fabs(targetDrone.pose.position.x) < 0.2
                        and math.fabs(targetDrone.pose.position.y) < 0.2
                        and math.fabs(targetDrone.pose.position.z) < 0.2
                        and math.fabs(euler[2]) < math.radians(20)):
                        if self.goalIndex < len(self.goals) - 1:
                            self.goalIndex += 1
                            if type(self.goals[self.goalIndex]) is str:
                                msg = Twist()
                                for i in range(0, 1000):
                                    self.pubCmd.publish(msg)
                                self.setFlightAnimation(8, 0)
                                rospy.sleep(1.0)
                                #for i in range(0, 1000):
                                #    self.pubCmd.publish(msg)
                                #rospy.sleep(1.5)
                                self.goalIndex += 1
                            rospy.loginfo("Next Goal (X,Y,Z,Yaw): " + str(self.goals[self.goalIndex]))
                        else:
                            pass
                            #self.action = Controller.ActionLand

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    controller = Controller()
    controller.run()
