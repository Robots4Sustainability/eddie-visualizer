#!/usr/bin/env python3
"""
Cartesian Error Visualizer for ROS2 Jazzy
Subscribes to Twist messages on cartesian error topics and plots them in real-time.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np

DEFAULT_POSITION_DEADBAND = 0.005 # 5 millimeters
DEFAULT_ROTATION_DEADBAND = 0.02  # ~1 degree


class CartesianErrorVisualizer(Node):
    def __init__(self, max_points=500):
        super().__init__('cartesian_error_visualizer')
        # Always initialize deadbands to defaults first
        self.position_deadband = DEFAULT_POSITION_DEADBAND
        self.rotation_deadband = DEFAULT_ROTATION_DEADBAND
        # Store PID gains
        self.pid_gains = {}
        
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

        self.get_logger().info("Waiting for interface node parameters...")
        # Create a client to talk to the parameter server of another node
        self.param_client = AsyncParameterClient(self, 'eddie_ros_interface')
        if not self.param_client.wait_for_services(timeout_sec=5.0):
            self.get_logger().error("Parameter service for 'eddie_ros_interface' not available. Using default deadbands.")
            self.position_deadband = DEFAULT_POSITION_DEADBAND # Fallback defaults
            self.rotation_deadband = DEFAULT_ROTATION_DEADBAND
        else:
            self.get_logger().info("Fetching PID deadband parameters asynchronously...")
            future_pos = self.param_client.get_parameters(['pid.right.pos.deadband'])
            future_rot = self.param_client.get_parameters(['pid.right.rot.deadband'])
            # Fetch all PID gains
            pid_param_names = []
            for arm in ['right', 'left']:
                for typ in ['pos', 'rot']:
                    for axis in ['x', 'y', 'z']:
                        for gain in ['p', 'i', 'd']:
                            pid_param_names.append(f'pid.{arm}.{typ}.{axis}.{gain}')
            future_gains = self.param_client.get_parameters(pid_param_names)
            def pos_done(fut):
                params = fut.result().values
                if params:
                    self.position_deadband = params[0].double_value
                    self.get_logger().info(f"Using Position Deadband: {self.position_deadband}")
                else:
                    self.position_deadband = DEFAULT_POSITION_DEADBAND
                    self.get_logger().warn("Could not fetch position deadband, using default.")
            def rot_done(fut):
                params = fut.result().values
                if params:
                    self.rotation_deadband = params[0].double_value
                    self.get_logger().info(f"Using Rotation Deadband: {self.rotation_deadband}")
                else:
                    self.rotation_deadband = DEFAULT_ROTATION_DEADBAND
                    self.get_logger().warn("Could not fetch rotation deadband, using default.")
            def gains_done(fut):
                params = fut.result().values
                for i, name in enumerate(pid_param_names):
                    self.pid_gains[name] = params[i].double_value if i < len(params) else None
                self.get_logger().info(f"Fetched PID gains: {self.pid_gains}")
            future_pos.add_done_callback(pos_done)
            future_rot.add_done_callback(rot_done)
            future_gains.add_done_callback(gains_done)
        
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
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.12)  # Reserve space at the bottom for PID gains
    
    def update_plot(frame):
        """Animation update function"""
        rclpy.spin_once(visualizer, timeout_sec=0)

        if hasattr(update_plot, 'pos_deadband_span_x'):
            update_plot.pos_deadband_span_x.remove()
        if hasattr(update_plot, 'pos_deadband_span_y'):
            update_plot.pos_deadband_span_y.remove()
        if hasattr(update_plot, 'pos_deadband_span_z'):
            update_plot.pos_deadband_span_z.remove()

        update_plot.pos_deadband_span_x = ax_right_linear_x.axhspan(-visualizer.position_deadband, visualizer.position_deadband, facecolor='green', alpha=0.2, label='Position Deadband')
        update_plot.pos_deadband_span_y = ax_right_linear_y.axhspan(-visualizer.position_deadband, visualizer.position_deadband, facecolor='green', alpha=0.2, label='Position Deadband')
        update_plot.pos_deadband_span_z = ax_right_linear_z.axhspan(-visualizer.position_deadband, visualizer.position_deadband, facecolor='green', alpha=0.2, label='Position Deadband')

        
        # Update right arm linear plot
        if len(visualizer.right_time) > 0:
            line_right_x.set_data(list(visualizer.right_time), list(visualizer.right_linear_x))
            line_right_y.set_data(list(visualizer.right_time), list(visualizer.right_linear_y))
            line_right_z.set_data(list(visualizer.right_time), list(visualizer.right_linear_z))
            
            for ax in axes:
                ax.relim()
                ax.autoscale_view()

        # Draw PID gains as small text in the reserved blank section below the plots
        pid_text = ''
        for arm in ['right', 'left']:
            for typ in ['pos', 'rot']:
                pid_text += f'{arm} {typ}: '
                for axis in ['x', 'y', 'z']:
                    for gain in ['p', 'i', 'd']:
                        key = f'pid.{arm}.{typ}.{axis}.{gain}'
                        val = visualizer.pid_gains.get(key, None)
                        if val is not None:
                            pid_text += f'{axis}.{gain}={val:.3f} '
                pid_text += '\n'
        # Remove previous text if any
        if hasattr(update_plot, 'pid_text_box'):
            update_plot.pid_text_box.remove()
        # Add new text box in reserved area (bottom center)
        update_plot.pid_text_box = fig.text(0.5, 0.01, pid_text, fontsize=8, color='gray', ha='center', va='bottom', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
        
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
