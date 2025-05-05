import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg._twist import Twist
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg import Vector3
from time import sleep

class move_path(Node):
 
  def __init__(self):
    super().__init__('hi')
    self.subscriber = self.create_subscription(LaserScan, 'scan', self.follow_wall, 10)
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

  def turn(self, left, time):
    print("function to turn left")

  def angle_to_wall(self, straight, front):
    print("finding angle to the wall")

  def follow_wall(self, msg):
    print("Basically just going straight")

def main(args=None):
  rclpy.init(args=args)

  print("running")
  node = move_path()

  while rclpy.ok():
    rclpy.spin_once(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()