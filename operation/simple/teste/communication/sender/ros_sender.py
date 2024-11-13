import rospy
from geometry_msgs.msg import Twist

from .sender import Sender


class RosSender(Sender):
    def __init__(self):
        
        rospy.init_node('bob_joga_bola')

        self.pub_robot1 = rospy.Publisher('/bob1/cmd_vel', Twist, queue_size=10)
        self.pub_robot2 = rospy.Publisher('/bob2/cmd_vel', Twist, queue_size=10)
        self.pub_robot3 = rospy.Publisher('/bob3/cmd_vel', Twist, queue_size=10)

    def transmit_robot(self, robot_id, left_speed, right_speed):
        twist = Twist()
        twist.linear.x = -right_speed
        twist.linear.y = -left_speed

        pub_map = {
            0: self.pub_robot1,
            1: self.pub_robot2,
            2: self.pub_robot3
        }

        pub_map[robot_id].publish(twist)