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

  def check_largest_open_angle(self, msg):
    index = 0
    for range_index in range(75):
      if self.avg_ranges(msg.ranges[range_index:range_index+15]) > self.avg_ranges(msg.ranges[index:index+15]):
        index = range_index

    for range_index in range(270, 345):
      if self.avg_ranges(msg.ranges[range_index:range_index+15]) > self.avg_ranges(msg.ranges[index:index+15]):
        index = range_index


    if index > 180 and index <= 320:
      self.turn('right', (index + 10) * (math.pi / 180))
    elif index < 180 and index >= 40:
      self.turn('left', (index + 10) * (math.pi / 180))
      sleep(2.0)

  def avg_ranges(self, range):
    CLEARANCE = 0.3
    num_numbers = 0
    total = 0
    for value in range:
      if value and value > CLEARANCE:
        num_numbers += 1
        total += value
      else:
        return float('-inf')

    return num_numbers / total

  def check_triangle_and_turn(self, msg):
    BUFFER_DIST = 0.4
    FRONT_CONE_ANGLE = 45

    left_cone = msg.ranges[0:FRONT_CONE_ANGLE]
    right_cone = msg.ranges[360 - FRONT_CONE_ANGLE:360]

    left_close = any(r < BUFFER_DIST and r > 0.05 for r in left_cone)
    right_close = any(r < BUFFER_DIST and r > 0.05 for r in right_cone)

    if left_close and not right_close:
        self.curved_turn("right")
        print("Curved turn RIGHT to avoid left obstacle")

    elif right_close and not left_close:
        self.curved_turn("left")
        print("Curved turn LEFT to avoid right obstacle")

    elif left_close and right_close:
        left_avg = self.avg_ranges(left_cone)
        right_avg = self.avg_ranges(right_cone)
        if left_avg > right_avg:
            self.curved_turn("left")
        else:
            self.curved_turn("right")
        print("Curved turn to avoid both sides")

  def curved_turn(self, direction):
    twist = Twist()
    twist.linear.x = 0.05  # slow forward motion

    if direction == "left":
        twist.angular.z = 0.5
    elif direction == "right":
        twist.angular.z = -0.5

    self.publisher.publish(twist)
    sleep(0.5)

    # Optional stop after curve
    stop_msg = Twist()
    stop_msg.linear = self.stop()
    stop_msg.angular = self.stop()
    self.publisher.publish(stop_msg)
    sleep(0.2)

  def follow_wall(self, msg):
    print("Basically just going straight")

    go_msg = Twist()
    go_msg.linear = self.go()
    go_msg.angular = self.stop()

    stop_msg = Twist()
    stop_msg.linear = self.stop()
    stop_msg.angular = self.stop()

    print("checking for open pathways and adjusting if necessary")
    self.check_largest_open_angle(msg)
    self.check_triangle_and_turn(msg)

    FRONT = 0.3
    if self.avg_ranges(msg.ranges[0:10] + msg.ranges[349:]) < FRONT:
      self.right_or_left(msg)

    self.publisher.publish(go_msg)
    sleep(0.5)


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