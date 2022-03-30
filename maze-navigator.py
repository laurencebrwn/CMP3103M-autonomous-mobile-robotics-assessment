import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = .04
robot_radius = .115

class CommandVelocity():
    def __init__(self):
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size= 1)
        self.wheel_sub = rospy.Subscriber('/wheel_vel_left', Float32, self.leftwheel_callback)

    def leftwheel_callback(self, msg):
        (v, a) = forward_kinematics(msg.data, 0.0)
        
        t = Twist()
        t.linear.x = v
        t.angular.z = a
        self.pub.publish(t)

# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a)
    c_r = v + (robot_radius * a)
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)


rospy.init_node('kinematics')
commander = CommandVelocity()
rospy.spin()
