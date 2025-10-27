#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
import math
import time
import tkinter as tk
from tkinter import messagebox
import threading

class PatternSelectorGUI(Node):
    def __init__(self):
        super().__init__('pattern_selector_gui')
        
        # Publishers for two turtles
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Flag to stop drawing
        self.stop_drawing = False
        
        # Spawn second turtle
        self.spawn_turtle()
        
        # Set pen colors
        self.set_pen_color('turtle1', 255, 0, 0)  # Red
        self.set_pen_color('turtle2', 0, 0, 255)  # Blue
        
        time.sleep(1)
        
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
        
    def create_gui(self):
        """Create the GUI window with pattern buttons"""
        self.root = tk.Tk()
        self.root.title("Multiturtle Pattern Selector")
        self.root.geometry("400x300")
        
        # Title label
        title_label = tk.Label(self.root, text="Select a Pattern to Draw", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Pattern buttons
        circle_btn = tk.Button(self.root, text="🔵 Draw Circles", command=self.draw_circles_thread, 
                               width=20, height=2, bg="lightblue", font=("Arial", 12))
        circle_btn.pack(pady=5)
        
        square_btn = tk.Button(self.root, text="🔲 Draw Squares", command=self.draw_squares_thread, 
                               width=20, height=2, bg="lightgreen", font=("Arial", 12))
        square_btn.pack(pady=5)
        
        triangle_btn = tk.Button(self.root, text="△ Draw Triangles", command=self.draw_triangles_thread, 
                                 width=20, height=2, bg="lightyellow", font=("Arial", 12))
        triangle_btn.pack(pady=5)
        
        spiral_btn = tk.Button(self.root, text="🌀 Draw Spirals", command=self.draw_spirals_thread, 
                               width=20, height=2, bg="lightcoral", font=("Arial", 12))
        spiral_btn.pack(pady=5)
        
        # Stop button
        stop_btn = tk.Button(self.root, text="⏹ Stop Drawing", command=self.stop_drawing_action, 
                             width=20, height=2, bg="red", fg="white", font=("Arial", 12))
        stop_btn.pack(pady=10)
        
        # Status label
        self.status_label = tk.Label(self.root, text="Ready to draw", font=("Arial", 10))
        self.status_label.pack(pady=5)
        
        self.root.mainloop()
        
    def draw_circles_thread(self):
        """Draw circles in a separate thread"""
        self.stop_drawing = False
        thread = threading.Thread(target=self.draw_circles)
        thread.daemon = True
        thread.start()
        
    def draw_squares_thread(self):
        """Draw squares in a separate thread"""
        self.stop_drawing = False
        thread = threading.Thread(target=self.draw_squares)
        thread.daemon = True
        thread.start()
        
    def draw_triangles_thread(self):
        """Draw triangles in a separate thread"""
        self.stop_drawing = False
        thread = threading.Thread(target=self.draw_triangles)
        thread.daemon = True
        thread.start()
        
    def draw_spirals_thread(self):
        """Draw spirals in a separate thread"""
        self.stop_drawing = False
        thread = threading.Thread(target=self.draw_spirals)
        thread.daemon = True
        thread.start()
        
    def draw_circles(self):
        """Draw synchronized circles"""
        self.get_logger().info('Drawing circles...')
        self.status_label.config(text="Drawing circles...")
        self.root.update()
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            duration = 6.28  # Time for one circle
            
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
            
            # Stop both turtles
            stop_msg = Twist()
            self.pub_turtle1.publish(stop_msg)
            self.pub_turtle2.publish(stop_msg)
            time.sleep(0.3)
        
        self.status_label.config(text="Ready to draw")
        self.root.update()
        
    def draw_squares(self):
        """Draw synchronized squares"""
        self.get_logger().info('Drawing squares...')
        self.status_label.config(text="Drawing squares...")
        self.root.update()
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            # Each side of the square
            for side in range(4):
                if self.stop_drawing:
                    break
                    
                # Move forward
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
                
                # Turn right (90 degrees)
                turn_msg1 = Twist()
                turn_msg1.linear.x = 0.0
                turn_msg1.angular.z = -1.57  # -90 degrees
                
                turn_msg2 = Twist()
                turn_msg2.linear.x = 0.0
                turn_msg2.angular.z = 1.57  # +90 degrees (opposite direction)
                
                start_time = time.time()
                while time.time() - start_time < 1.0 and not self.stop_drawing:
                    self.pub_turtle1.publish(turn_msg1)
                    self.pub_turtle2.publish(turn_msg2)
                    time.sleep(0.1)
            
            time.sleep(0.3)
        
        self.status_label.config(text="Ready to draw")
        self.root.update()
        
    def draw_triangles(self):
        """Draw synchronized triangles"""
        self.get_logger().info('Drawing triangles...')
        self.status_label.config(text="Drawing triangles...")
        self.root.update()
        
        for i in range(4):
            if self.stop_drawing:
                break
                
            # Each side of the triangle
            for side in range(3):
                if self.stop_drawing:
                    break
                    
                # Move forward
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
                
                # Turn right (120 degrees for triangle)
                turn_msg1 = Twist()
                turn_msg1.linear.x = 0.0
                turn_msg1.angular.z = -2.09  # -120 degrees
                
                turn_msg2 = Twist()
                turn_msg2.linear.x = 0.0
                turn_msg2.angular.z = 2.09  # +120 degrees
                
                start_time = time.time()
                while time.time() - start_time < 1.0 and not self.stop_drawing:
                    self.pub_turtle1.publish(turn_msg1)
                    self.pub_turtle2.publish(turn_msg2)
                    time.sleep(0.1)
            
            time.sleep(0.3)
        
        self.status_label.config(text="Ready to draw")
        self.root.update()
        
    def draw_spirals(self):
        """Draw synchronized spirals"""
        self.get_logger().info('Drawing spirals...')
        self.status_label.config(text="Drawing spirals...")
        self.root.update()
        
        duration = 8.0
        start_time = time.time()
        
        while time.time() - start_time < duration and not self.stop_drawing:
            # Increasing linear velocity and constant angular velocity = spiral
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
        
        # Stop both turtles
        stop_msg = Twist()
        self.pub_turtle1.publish(stop_msg)
        self.pub_turtle2.publish(stop_msg)
        
        self.status_label.config(text="Ready to draw")
        self.root.update()
        
    def stop_drawing_action(self):
        """Stop the current drawing"""
        self.stop_drawing = True
        self.get_logger().info('Drawing stopped!')
        self.status_label.config(text="Drawing stopped")
        self.root.update()

def main(args=None):
    rclpy.init(args=args)
    node = PatternSelectorGUI()
    
    # Create and run GUI
    node.create_gui()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

