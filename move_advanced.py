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

  def go(self):
    move_forward = Vector3()
    move_forward.x = 0.1
    move_forward.y = 0.1
    move_forward.z = 0.1
    return move_forward
  
  def stop(self):
    stop = Vector3()
    stop.x = 0.0
    stop.y = 0.0
    stop.z = 0.0
    return stop
  
  def turn(self, direction, time):
    go_msg = Twist()
    go_msg.linear = self.stop()
    go_msg.angular = self.go()
    if direction == "left":
      go_msg.angular.z = 1.0
    elif direction == "right":
      go_msg.angular.z = -1.0

    print(f"Turning {direction}!")
    self.publisher.publish(go_msg)
    sleep(time)

    stop_msg = Twist()
    stop_msg.linear = self.stop()
    stop_msg.angular = self.stop()
    self.publisher.publish(stop_msg)
    sleep(0.5)

  def angle_to_wall(self, leftahead, left, leftbehind):
    theta_rad = math.atan((leftahead - leftbehind) / (leftahead + leftbehind))

    if math.abs(theta_rad) > 0.25:
      # stop the robot
      stop_msg = Twist()
      stop_msg.linear = self.stop()
      stop_msg.angular = self.stop()
      self.publisher.publish(stop_msg)
      sleep(0.5)

      # turn left if angle is positive, right if angle is negative
      self.turn("left" if theta_rad > 0 else "right", theta_rad)


    print(f"Original wall angle: {math.degrees(theta_rad)} degrees")


  def follow_wall(self, msg):
    leftahead = min(msg.ranges[430:470])
    left = min(msg.ranges[520:560])
    leftbehind = min(msg.ranges[610:650])
    print(f"Current wall angle: {math.degrees(math.atan((leftahead - leftbehind) / (leftahead + leftbehind)))} degrees")

    go_msg = Twist()
    go_msg.linear = self.go()
    go_msg.angular = self.stop()

    stop_msg = Twist()
    stop_msg.linear = self.stop()
    stop_msg.angular = self.stop()

    FRONT = min(msg.ranges[340:380])
    if FRONT < 0.2:
      self.publisher.publish(stop_msg)
      sleep(0.5)
      self.turn("right", 1.5708)
    elif (leftbehind / left) < (math.sqrt(2) * 0.9) and left > 0.5 and leftahead > (leftbehind * 1.1):
      self.publisher.publish(stop_msg)
      sleep(0.5)
      self.turn("left", 1.5708)
    else:
      self.publisher.publish(go_msg)

    def is_between(self, num, min, max):
      return num >= min and num <= max

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