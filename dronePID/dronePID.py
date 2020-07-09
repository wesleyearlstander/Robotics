import rospy
import numpy as np
import math
import sys
import tf.transformations
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

def eulerToQuaternion(euler):
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def quaternionToEuler(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    q = (x,y,z,w)
    return tf.transformations.euler_from_quaternion(q)

def angle(p1, p2):
    delta_y = p2[1]-p1[1]
    delta_x = p2[0]-p1[0]
    return math.atan2(delta_y, delta_x)

class controller:
    Kp = 1
    Ki = 1
    Kd = 1

    def setupGain (self, p, i, d):
        self.Kp = p
        self.Ki = i
        self.Kd = d

    def controller(self, errorHistory, dt):
        p = self.Kp * errorHistory[len(errorHistory)-1]
        i = self.Ki * np.cumsum(errorHistory)[len(errorHistory)-1] * dt
        d = 0
        if len(errorHistory) > 1:
            d = self.Kd * (errorHistory[len(errorHistory)-1] - errorHistory[len(errorHistory)-2])/dt
        return p + i + d

class quadrotor:

    position = None
    rotation = None
    orientation = None
    destination = None
    time = 0
    positionErrorHistory = []
    rotationErrorHistory = []
    positionController = None
    rotationController = None

    def setDestination(self, xc, zy):
        self.destination = [xc,zy]

    def initialize(self, param):
        self.setDestination(float(param[1]), float(param[2]))
        self.positionController = controller()
        self.rotationController = controller()
        self.positionController.setupGain(10,0.5,5)
        self.rotationController.setupGain(15,0.5,0.1)
        rospy.init_node('droneController', anonymous=True)
        takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        move = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        empty = Empty()
        twist = Twist()
        while takeoff.get_num_connections() < 1:
            rospy.sleep(0.1)
        takeoff.publish(empty)
        while move.get_num_connections() < 1:
            rospy.sleep(0.1)
        rate = rospy.Rate(10) # 10hz or 0.1s
        while True:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str('quadrotor'), "")
            if resp_coordinates.success:
                self.orientation = resp_coordinates.pose.orientation
                self.rotation = quaternionToEuler(self.orientation)
                self.position = resp_coordinates.pose.position
                self.positionErrorHistory.append(self.calculatePositionError())
                self.rotationErrorHistory.append(self.calculateRotationError())
                #print self.errorHistory[len(self.errorHistory)-1]
                #print self.calculateRotationError()
                twist.linear.x = self.positionController.controller(self.positionErrorHistory, 0.1)
                twist.angular.z = self.rotationController.controller(self.rotationErrorHistory, 0.1)
                print twist.linear.x, twist.angular.z
                move.publish(twist)
            self.time += 0.1
            if self.calculatePositionError() < 0.75:
                break
            rate.sleep()
        twist.linear.x = 0
        twist.angular.z = 0
        move.publish(twist)
        while land.get_num_connections() < 1:
            rospy.sleep(0.1)
        land.publish(empty)

    def calculatePositionError(self):
        xErr = self.destination[0] - self.position.x
        yErr = self.destination[1] - self.position.y
        return math.sqrt(math.pow(xErr,2)+math.pow(yErr,2))

    def calculateRotationError(self):
        goalPos = [self.destination[0], self.destination[1]]
        currentPos = [self.position.x, self.position.y]
        goalRotation = angle(currentPos, goalPos)
        currentRotation = self.rotation[2]
        error = goalRotation - currentRotation
        if error > np.pi:
            error -= np.pi
            error = error - np.pi
        return error

    def show_quadrotor(self):
        try:
            e = quaternionToEuler(self.orientation)
            print str(e)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    try:
        quadrotor = quadrotor()
        quadrotor.initialize(sys.argv)
    except rospy.ROSInterruptException:
        pass
