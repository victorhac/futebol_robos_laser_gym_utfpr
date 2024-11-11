import rospy
from geometry_msgs.msg import Twist

from .sender import Sender

class RosSender(Sender):
    def __init__(self):
        rospy.init_node('keyboard_control')

        self.pub_robot1 = rospy.Publisher('/bob1/cmd_vel', Twist, queue_size=10)

    def transmit_robot(self, robot_id, left_speed, right_speed):
        twist = Twist()
        twist.linear.x = right_speed
        twist.linear.y = left_speed
        twist.linear.z = robot_id

        self.pub_robot1.publish(twist)