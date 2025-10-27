#!/usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class GUIPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher = self.create_publisher(String, 'drawing_command', 10)
        
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')

class PatternGUI:
    def __init__(self):
        rclpy.init()
        self.node = GUIPublisher()
        
        self.root = tk.Tk()
        self.root.title("Multiturtle Pattern Selector")
        self.root.geometry("400x300")
        
        # Title
        title_label = tk.Label(self.root, text="Select a Pattern to Draw", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Buttons
        tk.Button(self.root, text="🔵 Draw Circles", command=lambda: self.send_command('circles'), 
                  width=20, height=2, bg="lightblue", font=("Arial", 12)).pack(pady=5)
        
        tk.Button(self.root, text="🔲 Draw Squares", command=lambda: self.send_command('squares'), 
                  width=20, height=2, bg="lightgreen", font=("Arial", 12)).pack(pady=5)
        
        tk.Button(self.root, text="△ Draw Triangles", command=lambda: self.send_command('triangles'), 
                  width=20, height=2, bg="lightyellow", font=("Arial", 12)).pack(pady=5)
        
        tk.Button(self.root, text="🌀 Draw Spirals", command=lambda: self.send_command('spirals'), 
                  width=20, height=2, bg="lightcoral", font=("Arial", 12)).pack(pady=5)
        
        tk.Button(self.root, text="⏹ Stop Drawing", command=lambda: self.send_command('stop'), 
                  width=20, height=2, bg="red", fg="white", font=("Arial", 12)).pack(pady=10)
        
        # Status
        self.status_label = tk.Label(self.root, text="Ready", font=("Arial", 10))
        self.status_label.pack(pady=5)
        
    def send_command(self, command):
        self.status_label.config(text=f"Sending: {command}")
        self.root.update()
        self.node.publish_command(command)
        
    def run(self):
        def spin_ros():
            while True:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        
        spin_thread = threading.Thread(target=spin_ros, daemon=True)
        spin_thread.start()
        
        self.root.mainloop()
        rclpy.shutdown()

def main():
    gui = PatternGUI()
    gui.run()

if __name__ == '__main__':
    main()
