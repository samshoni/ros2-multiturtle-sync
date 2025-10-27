#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_msgs.msg import String
import time

class DrawingController(Node):
    def __init__(self):
        super().__init__('drawing_controller')
        
        # Publishers for two turtles
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Subscriber to receive drawing commands
        self.subscription = self.create_subscription(
            String,
            'drawing_command',
            self.command_callback,
            10)
        
        # Flag to stop drawing
        self.stop_drawing = False
        
        self.get_logger().info('Drawing Controller Node started!')
        
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
            pass
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 3
        request.off = 0
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    def command_callback(self, msg):
        """Callback to receive drawing commands"""
        command = msg.data.lower()
        
        if command == 'circles':
            self.draw_circles()
        elif command == 'squares':
            self.draw_squares()
        elif command == 'triangles':
            self.draw_triangles()
        elif command == 'spirals':
            self.draw_spirals()
        elif command == 'stop':
            self.stop_drawing = True
        
    def draw_circles(self):
        """Draw synchronized circles"""
        self.get_logger().info('Drawing circles...')
        self.stop_drawing = False
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            duration = 6.28
            
            msg1 = Twist()
            msg1.linear.x = 1.0
            msg1.angular.z = 1.0
            
            msg2 = Twist()
            msg2.linear.x = 1.0
            msg2.angular.z = -1.0
            
            start_time = time.time()
            while time.time() - start_time < duration and not self.stop_drawing:
                self.pub_turtle1.publish(msg1)
                self.pub_turtle2.publish(msg2)
                time.sleep(0.1)
            
            stop_msg = Twist()
            self.pub_turtle1.publish(stop_msg)
            self.pub_turtle2.publish(stop_msg)
            time.sleep(0.3)
        
        self.get_logger().info('Circles drawing complete!')
        
    def draw_squares(self):
        """Draw synchronized squares"""
        self.get_logger().info('Drawing squares...')
        self.stop_drawing = False
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            for side in range(4):
                if self.stop_drawing:
                    break
                    
                msg1 = Twist()
                msg1.linear.x = 1.5
                msg1.angular.z = 0.0
                
                msg2 = Twist()
                msg2.linear.x = 1.5
                msg2.angular.z = 0.0
                
                start_time = time.time()
                while time.time() - start_time < 1.5 and not self.stop_drawing:
                    self.pub_turtle1.publish(msg1)
                    self.pub_turtle2.publish(msg2)
                    time.sleep(0.1)
                
                turn_msg1 = Twist()
                turn_msg1.linear.x = 0.0
                turn_msg1.angular.z = -1.57
                
                turn_msg2 = Twist()
                turn_msg2.linear.x = 0.0
                turn_msg2.angular.z = 1.57
                
                start_time = time.time()
                while time.time() - start_time < 1.0 and not self.stop_drawing:
                    self.pub_turtle1.publish(turn_msg1)
                    self.pub_turtle2.publish(turn_msg2)
                    time.sleep(0.1)
            
            time.sleep(0.3)
        
        self.get_logger().info('Squares drawing complete!')
        
    def draw_triangles(self):
        """Draw synchronized triangles"""
        self.get_logger().info('Drawing triangles...')
        self.stop_drawing = False
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            for side in range(3):
                if self.stop_drawing:
                    break
                    
                msg1 = Twist()
                msg1.linear.x = 1.5
                msg1.angular.z = 0.0
                
                msg2 = Twist()
                msg2.linear.x = 1.5
                msg2.angular.z = 0.0
                
                start_time = time.time()
                while time.time() - start_time < 1.5 and not self.stop_drawing:
                    self.pub_turtle1.publish(msg1)
                    self.pub_turtle2.publish(msg2)
                    time.sleep(0.1)
                
                turn_msg1 = Twist()
                turn_msg1.linear.x = 0.0
                turn_msg1.angular.z = -2.09
                
                turn_msg2 = Twist()
                turn_msg2.linear.x = 0.0
                turn_msg2.angular.z = 2.09
                
                start_time = time.time()
                while time.time() - start_time < 1.0 and not self.stop_drawing:
                    self.pub_turtle1.publish(turn_msg1)
                    self.pub_turtle2.publish(turn_msg2)
                    time.sleep(0.1)
            
            time.sleep(0.3)
        
        self.get_logger().info('Triangles drawing complete!')
        
    def draw_spirals(self):
        """Draw synchronized spirals"""
        self.get_logger().info('Drawing spirals...')
        self.stop_drawing = False
        
        duration = 8.0
        start_time = time.time()
        
        while time.time() - start_time < duration and not self.stop_drawing:
            elapsed = time.time() - start_time
            linear_vel = 1.0 + (elapsed / duration)
            
            msg1 = Twist()
            msg1.linear.x = linear_vel
            msg1.angular.z = 1.5
            
            msg2 = Twist()
            msg2.linear.x = linear_vel
            msg2.angular.z = -1.5
            
            self.pub_turtle1.publish(msg1)
            self.pub_turtle2.publish(msg2)
            time.sleep(0.1)
        
        stop_msg = Twist()
        self.pub_turtle1.publish(stop_msg)
        self.pub_turtle2.publish(stop_msg)
        
        self.get_logger().info('Spirals drawing complete!')

def main(args=None):
    rclpy.init(args=args)
    node = DrawingController()
    
    # Spawn turtle and set colors before spinning
    node.spawn_turtle()
    node.set_pen_color('turtle1', 255, 0, 0)  # Red
    node.set_pen_color('turtle2', 0, 0, 255)  # Blue
    
    time.sleep(1)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
