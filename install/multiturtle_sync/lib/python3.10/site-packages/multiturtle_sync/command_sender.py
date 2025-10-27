#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('command_sender')
    publisher = node.create_publisher(String, 'drawing_command', 10)
    
    print("\n" + "="*50)
    print("Multiturtle Pattern Selector")
    print("="*50)
    print("\nAvailable commands:")
    print("  1 - Draw Circles")
    print("  2 - Draw Squares")
    print("  3 - Draw Triangles")
    print("  4 - Draw Spirals")
    print("  s - Stop Drawing")
    print("  q - Quit")
    print("="*50 + "\n")
    
    commands_map = {
        '1': 'circles',
        '2': 'squares',
        '3': 'triangles',
        '4': 'spirals',
        's': 'stop',
    }
    
    try:
        while True:
            choice = input("Enter command (1-4, s, or q): ").strip().lower()
            
            if choice == 'q':
                print("Exiting...")
                break
            
            if choice in commands_map:
                msg = String()
                msg.data = commands_map[choice]
                publisher.publish(msg)
                print(f"✓ Sent command: {commands_map[choice]}\n")
            else:
                print("Invalid command. Please try again.\n")
    
    except KeyboardInterrupt:
        print("\n\nExiting...")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
