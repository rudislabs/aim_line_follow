import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector

class LineFollow(Node):

	def __init__(self):
		super().__init__('aim_line_follow')

		# Subscribers
		self.pixy_subscriber = self.create_subscription(
			PixyVector,
			'/cupcar0/PixyVector',
			self.listener_callback,
			10)

		# Publishers
		self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

		self.speed_vector = Vector3()
		self.steer_vector = Vector3()
		self.cmd_vel = Twist()

		# Timer setup
		# timer_period = 0.5 #seconds
		# self.timer = self.create_timer(timer_period, self.timer_callback)
		# self.i = 0

	def get_num_vectors(self, msg):
		num_vectors = 0
		if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
			num_vectors = num_vectors + 1
		if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
			num_vectors = num_vectors + 1
		return num_vectors

	# def timer_callback(self):
	# 	#TODO

	def listener_callback(self, msg):
		#TODO
		frame_width = 79
		frame_height = 52
		window_center = (frame_width / 2)
		x = 0
		y = 0
		steer = 0
		speed = 0
		num_vectors = self.get_num_vectors(msg)

		if(num_vectors == 0):
			steer = 0
			speed = 0

		if(num_vectors == 1):
			if(msg.m0_x1 > msg.m0_x0):
				x = (msg.m0_x1 - msg.m0_x0) / frame_width
				y = (msg.m0_y1 - msg.m0_y0) / frame_height
			else:
				x = (msg.m0_x0 - msg.m0_x1) / frame_width
				y = (msg.m0_y0 - msg.m0_y1) / frame_height
			if(msg.m0_x0 != msg.m0_x1):
				steer = (-1) * (x / y)
				speed = 1.1
			else:
				steer = 0
				speed = 1.0

		if(num_vectors == 2):
			m_x1 = (msg.m0_x1 + msg.m1_x1) / 2
			steer = (m_x1 - window_center) / frame_width
			speed = 1.1

		self.speed_vector.x = float(speed)
		self.steer_vector.z = float(steer)

		self.cmd_vel.linear = self.speed_vector
		self.cmd_vel.angular = self.steer_vector

		self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
	rclpy.init(args=args)

	line_follow = LineFollow()

	rclpy.spin(line_follow)

	line_follow.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
