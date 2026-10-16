#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDController(Node):

  def __init__(self, kp, ki = 0, kd = 0):
    super().__init__("p_controller")
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = None
    self.state = None
    self.proportional_action = 0
    
    self.integral_error = 0.0
    self.max_vel = 0.2
    self.previous_error = None
    self.previous_time = None

    self.actuation_pub = self.create_publisher(Float64, "control_effort", 1)
    self.dist_set_point_sub = self.create_subscription(Float64, "setpoint", self.setpoint_cb, 1)
    self.dist_state_sub = self.create_subscription(Float64, "state", self.state_cb, 1)

  def setpoint_cb(self, msg):
    self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
    self.reset()
    self.setpoint = msg.data

  def state_cb(self, msg):
    if self.setpoint == None:
      return
    self.state = msg.data
    error = self.setpoint - self.state

    now = self.get_clock().now().nanoseconds / 1e9
    if self.previous_time is None:
      self.previous_time = now
      self.previous_error = error
    dt = now - self.previous_time

    # Proportional
    p_actuation = self.kp*error

    # Integrative
    if dt > 0:
      self.integral_error += error * dt
    i_actuation = self.ki * self.integral_error

    # Derivative
    if dt > 0 and self.previous_error is not None:
      derivative = (error - self.previous_error) / dt
    else:
      derivative = 0.0
    d_actuation = self.kd * derivative

    # Actuation
    actuation = p_actuation + i_actuation + d_actuation

    # Message sending
    msg = Float64()
    msg.data = actuation
    self.actuation_pub.publish(msg)

  def reset( self ):
    self.setpoint = None
    self.state = None
    self.integral_error = 0.0
    self.previous_error = None
    self.previous_time = None

def main():
  rclpy.init()
  p_ctrl = PIDController(0.5)
  rclpy.spin(p_ctrl)

if __name__ == "__main__":
    main()