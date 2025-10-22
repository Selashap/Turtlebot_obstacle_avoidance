import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class robot_movement(Node):
	def __init__(self):
		super().__init__('robot_movement')
		self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.LScan_sub = self.create_subscription(LaserScan, '/scan', self.Lidar_callback, 10)
		self.start_time = time.time()
	
	def Lidar_callback(self, msg):
		self.cmd_msg = Twist()
				
		remaining_time = time.time() - self.start_time
		
		if remaining_time >= 60.0:
			self.stop_robot()
		else:
			self.robot_move(msg)
		
	def stop_robot(self):
		self.cmd_msg.linear.x = 0.0
		self.cmd_msg.angular.z = 0.0
		self.cmd_pub.publish(self.cmd_msg)
		rclpy.shutdown()
		
	def robot_move(self,msg):
		rob_front_view =  msg.ranges[int(len(msg.ranges)//4)]
		rob_Left_view =  msg.ranges[0] 
		rob_mid_Left_view =  msg.ranges[int(1*len(msg.ranges))//10]
		rob_mid_right_view =  msg.ranges[int(3*len(msg.ranges))//8]
		rob_right_view =  msg.ranges[int(len(msg.ranges)//2)]
		
# the front view of the robot is in the first quarter of the lidar range, therefore I devide the value by 4. and it relates to all sides (left, right)
	
		if rob_front_view < 0.5:
			print('front obstacle')
			self.turn_right()
			
		elif rob_mid_right_view < 0.5:
			print('mid right obstacle')
			self.turn_left()
				
		elif rob_right_view < 0.5:
			print('right obstacle')
			self.turn_left()
		
		elif rob_mid_Left_view < 0.5:
			print('mid left obstacle')
			self.turn_right()
						
		elif rob_Left_view < 0.5:
			print('left obstacle')
			self.turn_right()
				
		else:
			print('no obstacle')
			self.drive_forward()
			
	def turn_left(self):
		self.cmd_msg.linear.x = 0.0
		self.cmd_msg.angular.z = -1.0
		self.cmd_pub.publish(self.cmd_msg)
	
	def turn_right(self):
		self.cmd_msg.linear.x = 0.0
		self.cmd_msg.angular.z = 1.0
		self.cmd_pub.publish(self.cmd_msg)
		
	def drive_forward(self):
		self.cmd_msg.linear.x = 1.0
		self.cmd_msg.angular.z = 0.0
		self.cmd_pub.publish(self.cmd_msg)
		
def main(args=None):
	rclpy.init(args=args)
	robot_movement_Node = robot_movement()
	rclpy.spin(robot_movement_Node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
