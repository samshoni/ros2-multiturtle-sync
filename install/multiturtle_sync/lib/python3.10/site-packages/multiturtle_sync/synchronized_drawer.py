#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
import math
import time

class SynchronizedDrawer(Node):
    def __init__(self):
        super().__init__('synchronized_drawer')
        
        # Publishers for two turtles
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Spawn second turtle
        self.spawn_turtle()
        
        # Set pen colors
        self.set_pen_color('turtle1', 255, 0, 0)  # Red
        self.set_pen_color('turtle2', 0, 0, 255)  # Blue
        
        # Wait a bit for setup
        time.sleep(1)
        
        # Start drawing
        self.draw_synchronized_pattern()
        
    def spawn_turtle(self):
        """Spawn a second turtle"""
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x = 8.0
        request.y = 5.5
        request.theta = 0.0
        request.name = 'turtle2'
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Turtle2 spawned successfully!')
        
    def set_pen_color(self, turtle_name, r, g, b):
        """Set pen color for a turtle"""
        client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for set_pen service for {turtle_name}...')
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 3
        request.off = 0
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    def move_turtle(self, publisher, linear, angular, duration):
        """Move a turtle with given velocities for a duration"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            publisher.publish(msg)
            time.sleep(0.1)
            
        # Stop the turtle
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        publisher.publish(msg)
        
    def draw_synchronized_pattern(self):
        """Draw synchronized circles"""
        self.get_logger().info('Starting synchronized drawing...')
        
        # Draw 4 synchronized circles
        for i in range(4):
            self.get_logger().info(f'Drawing circle {i+1}/4')
            
            # Both turtles draw circles simultaneously
            # Turtle1 goes clockwise, Turtle2 goes counter-clockwise
            duration = 6.28  # Time for one circle (2*pi seconds at speed 1)
            
            msg1 = Twist()
            msg1.linear.x = 1.0
            msg1.angular.z = 1.0  # Clockwise
            
            msg2 = Twist()
            msg2.linear.x = 1.0
            msg2.angular.z = -1.0  # Counter-clockwise
            
            start_time = time.time()
            while time.time() - start_time < duration:
                self.pub_turtle1.publish(msg1)
                self.pub_turtle2.publish(msg2)
                time.sleep(0.1)
            
            # Stop both turtles
            stop_msg = Twist()
            self.pub_turtle1.publish(stop_msg)
            self.pub_turtle2.publish(stop_msg)
            time.sleep(0.5)
        
        self.get_logger().info('Synchronized drawing complete!')

def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedDrawer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
