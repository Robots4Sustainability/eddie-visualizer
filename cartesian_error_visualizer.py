#!/usr/bin/env python3
"""
Cartesian Error Visualizer for ROS2 Jazzy
Subscribes to Twist messages on cartesian error topics and plots them in real-time.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np


class CartesianErrorVisualizer(Node):
    def __init__(self, max_points=500):
        super().__init__('cartesian_error_visualizer')
        
        # Maximum number of points to display
        self.max_points = max_points
        
        # Data storage for right arm
        self.right_time = deque(maxlen=max_points)
        self.right_linear_x = deque(maxlen=max_points)
        self.right_linear_y = deque(maxlen=max_points)
        self.right_linear_z = deque(maxlen=max_points)
        self.right_angular_x = deque(maxlen=max_points)
        self.right_angular_y = deque(maxlen=max_points)
        self.right_angular_z = deque(maxlen=max_points)
        
        # Data storage for left arm
        self.left_time = deque(maxlen=max_points)
        self.left_linear_x = deque(maxlen=max_points)
        self.left_linear_y = deque(maxlen=max_points)
        self.left_linear_z = deque(maxlen=max_points)
        self.left_angular_x = deque(maxlen=max_points)
        self.left_angular_y = deque(maxlen=max_points)
        self.left_angular_z = deque(maxlen=max_points)
        
        # Time tracking
        self.start_time = self.get_clock().now()
        
        # Create subscribers
        self.right_sub = self.create_subscription(
            Twist,
            'right_arm/cartesian_error',
            self.right_callback,
            10
        )
        self.left_sub = self.create_subscription(
            Twist,
            'left_arm/cartesian_error',
            self.left_callback,
            10
        )
        
        self.get_logger().info('Cartesian Error Visualizer started')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - right_arm/cartesian_error')
        self.get_logger().info('  - left_arm/cartesian_error')
    
    def get_elapsed_time(self):
        """Get elapsed time since node started in seconds"""
        current_time = self.get_clock().now()
        return (current_time - self.start_time).nanoseconds / 1e9
    
    def right_callback(self, msg):
        """Callback for right arm cartesian error"""
        current_time = self.get_elapsed_time()
        
        self.right_time.append(current_time)
        self.right_linear_x.append(msg.linear.x)
        self.right_linear_y.append(msg.linear.y)
        self.right_linear_z.append(msg.linear.z)
        self.right_angular_x.append(msg.angular.x)
        self.right_angular_y.append(msg.angular.y)
        self.right_angular_z.append(msg.angular.z)
    
    def left_callback(self, msg):
        """Callback for left arm cartesian error"""
        current_time = self.get_elapsed_time()
        
        self.left_time.append(current_time)
        self.left_linear_x.append(msg.linear.x)
        self.left_linear_y.append(msg.linear.y)
        self.left_linear_z.append(msg.linear.z)
        self.left_angular_x.append(msg.angular.x)
        self.left_angular_y.append(msg.angular.y)
        self.left_angular_z.append(msg.angular.z)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    visualizer = CartesianErrorVisualizer(max_points=500)
    
    # Set up the plot
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Cartesian Error Visualization', fontsize=16, fontweight='bold')

    ax_right_linear_x = axes[0]
    ax_right_linear_x.set_title('Right Arm - Linear Errors X')
    ax_right_linear_x.set_xlabel('Time (s)')
    ax_right_linear_x.set_ylabel('Error (m)')
    ax_right_linear_x.grid(True, alpha=0.3)
    ax_right_linear_x.axhline(y=0, color='black', linestyle='-', linewidth=1, alpha=0.5)
    line_right_x, = ax_right_linear_x.plot([], [], 'r-', label='X', linewidth=1.5)
    ax_right_linear_x.legend(loc='upper right')

    ax_right_linear_y = axes[1]
    ax_right_linear_y.set_title('Right Arm - Linear Errors Y')
    ax_right_linear_y.set_xlabel('Time (s)')
    ax_right_linear_y.set_ylabel('Error (m)')
    ax_right_linear_y.grid(True, alpha=0.3)
    ax_right_linear_y.axhline(y=0, color='black', linestyle='-', linewidth=1, alpha=0.5)
    line_right_y, = ax_right_linear_y.plot([], [], 'g-', label='Y', linewidth=1.5)
    ax_right_linear_y.legend(loc='upper right')

    ax_right_linear_z = axes[2]
    ax_right_linear_z.set_title('Right Arm - Linear Errors Z')
    ax_right_linear_z.set_xlabel('Time (s)')
    ax_right_linear_z.set_ylabel('Error (m)')
    ax_right_linear_z.grid(True, alpha=0.3)
    ax_right_linear_z.axhline(y=0, color='black', linestyle='-', linewidth=1, alpha=0.5)
    line_right_z, = ax_right_linear_z.plot([], [], 'b-', label='Z', linewidth=1.5)
    ax_right_linear_z.legend(loc='upper right')

    # Configure subplots
    # # Top left: Right arm linear errors
    # ax_right_linear = axes[0, 0]
    # ax_right_linear.set_title('Right Arm - Linear Errors (Position)')
    # ax_right_linear.set_xlabel('Time (s)')
    # ax_right_linear.set_ylabel('Error (m)')
    # ax_right_linear.grid(True, alpha=0.3)
    
    # # Top right: Left arm linear errors
    # ax_left_linear = axes[0, 1]
    # ax_left_linear.set_title('Left Arm - Linear Errors (Position)')
    # ax_left_linear.set_xlabel('Time (s)')
    # ax_left_linear.set_ylabel('Error (m)')
    # ax_left_linear.grid(True, alpha=0.3)
    
    # # Bottom left: Right arm angular errors
    # ax_right_angular = axes[1, 0]
    # ax_right_angular.set_title('Right Arm - Angular Errors (Orientation)')
    # ax_right_angular.set_xlabel('Time (s)')
    # ax_right_angular.set_ylabel('Error (rad)')
    # ax_right_angular.grid(True, alpha=0.3)
    
    # # Bottom right: Left arm angular errors
    # ax_left_angular = axes[1, 1]
    # ax_left_angular.set_title('Left Arm - Angular Errors (Orientation)')
    # ax_left_angular.set_xlabel('Time (s)')
    # ax_left_angular.set_ylabel('Error (rad)')
    # ax_left_angular.grid(True, alpha=0.3)
    
    # # Initialize line objects for right arm linear
    # line_right_x, = ax_right_linear.plot([], [], 'r-', label='X', linewidth=1.5)
    # line_right_y, = ax_right_linear.plot([], [], 'g-', label='Y', linewidth=1.5)
    # line_right_z, = ax_right_linear.plot([], [], 'b-', label='Z', linewidth=1.5)
    # ax_right_linear.legend(loc='upper right')
    
    # # Initialize line objects for left arm linear
    # line_left_x, = ax_left_linear.plot([], [], 'r-', label='X', linewidth=1.5)
    # line_left_y, = ax_left_linear.plot([], [], 'g-', label='Y', linewidth=1.5)
    # line_left_z, = ax_left_linear.plot([], [], 'b-', label='Z', linewidth=1.5)
    # ax_left_linear.legend(loc='upper right')
    
    # # Initialize line objects for right arm angular
    # line_right_roll, = ax_right_angular.plot([], [], 'r-', label='Roll', linewidth=1.5)
    # line_right_pitch, = ax_right_angular.plot([], [], 'g-', label='Pitch', linewidth=1.5)
    # line_right_yaw, = ax_right_angular.plot([], [], 'b-', label='Yaw', linewidth=1.5)
    # ax_right_angular.legend(loc='upper right')
    
    # # Initialize line objects for left arm angular
    # line_left_roll, = ax_left_angular.plot([], [], 'r-', label='Roll', linewidth=1.5)
    # line_left_pitch, = ax_left_angular.plot([], [], 'g-', label='Pitch', linewidth=1.5)
    # line_left_yaw, = ax_left_angular.plot([], [], 'b-', label='Yaw', linewidth=1.5)
    # ax_left_angular.legend(loc='upper right')
    
    plt.tight_layout()
    
    def update_plot(frame):
        """Animation update function"""
        # Spin the ROS2 node to process callbacks
        rclpy.spin_once(visualizer, timeout_sec=0)
        
        # Update right arm linear plot
        if len(visualizer.right_time) > 0:
            line_right_x.set_data(list(visualizer.right_time), list(visualizer.right_linear_x))
            line_right_y.set_data(list(visualizer.right_time), list(visualizer.right_linear_y))
            line_right_z.set_data(list(visualizer.right_time), list(visualizer.right_linear_z))
            
            ax_right_linear_x.relim()
            ax_right_linear_x.autoscale_view()
            ax_right_linear_y.relim()
            ax_right_linear_y.autoscale_view()
            ax_right_linear_z.relim()
            ax_right_linear_z.autoscale_view()

        #     # Auto-scale axes
        #     ax_right_linear.relim()
        #     ax_right_linear.autoscale_view()
        
        # # Update left arm linear plot
        # if len(visualizer.left_time) > 0:
        #     line_left_x.set_data(list(visualizer.left_time), list(visualizer.left_linear_x))
        #     line_left_y.set_data(list(visualizer.left_time), list(visualizer.left_linear_y))
        #     line_left_z.set_data(list(visualizer.left_time), list(visualizer.left_linear_z))
            
        #     # Auto-scale axes
        #     ax_left_linear.relim()
        #     ax_left_linear.autoscale_view()
        
        # # Update right arm angular plot
        # if len(visualizer.right_time) > 0:
        #     line_right_roll.set_data(list(visualizer.right_time), list(visualizer.right_angular_x))
        #     line_right_pitch.set_data(list(visualizer.right_time), list(visualizer.right_angular_y))
        #     line_right_yaw.set_data(list(visualizer.right_time), list(visualizer.right_angular_z))
            
        #     # Auto-scale axes
        #     ax_right_angular.relim()
        #     ax_right_angular.autoscale_view()
        
        # # Update left arm angular plot
        # if len(visualizer.left_time) > 0:
        #     line_left_roll.set_data(list(visualizer.left_time), list(visualizer.left_angular_x))
        #     line_left_pitch.set_data(list(visualizer.left_time), list(visualizer.left_angular_y))
        #     line_left_yaw.set_data(list(visualizer.left_time), list(visualizer.left_angular_z))
            
        #     # Auto-scale axes
        #     ax_left_angular.relim()
        #     ax_left_angular.autoscale_view()
        
        return (line_right_x, line_right_y, line_right_z)

        # return (line_right_x, line_right_y, line_right_z,
        #         line_left_x, line_left_y, line_left_z,
        #         line_right_roll, line_right_pitch, line_right_yaw,
        #         line_left_roll, line_left_pitch, line_left_yaw)
    
    # Create animation with 50ms interval (20 Hz)
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
