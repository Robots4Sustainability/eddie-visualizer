#!/usr/bin/env python3
"""
Cartesian Error Visualizer for ROS2 Jazzy
Subscribes to Twist messages on cartesian error topics and plots them in real-time.
Automatically detects parameter changes on 'eddie_ros_interface'.
Updates Deadband visuals and PID text dynamically
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterEvent
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

DEFAULT_POSITION_DEADBAND = 0.005 

class HybridVisualizer(Node):
    def __init__(self, max_points=200):
        super().__init__('cartesian_error_visualizer')
        
        # Configuration
        self.target_node_name = 'eddie_ros_interface'
        self.max_points = max_points
        
        # State Variables
        self.position_deadband = DEFAULT_POSITION_DEADBAND
        self.pid_gains = {}
        
        # Flags for updates
        self.params_need_fetch = True  # Fetch immediately on start
        self.visuals_need_update = True
        
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

        # Setup Parameter Client
        self.param_client = AsyncParameterClient(self, self.target_node_name)
        
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
        self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10
        )

        self.get_logger().info("Cartesian Error Visualizer started. Waiting for data...")

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

    def parameter_event_callback(self, msg: ParameterEvent):
        """Triggered when any parameter changes in the system."""
        if msg.node == f"/{self.target_node_name}":
            self.get_logger().info("Parameter change detected. Queuing update...")
            self.params_need_fetch = True

    def fetch_parameters_async(self):
        """Initiates an async request to get the latest values."""
        if not self.param_client.services_are_ready():
            return

        param_names = ['pid.right.pos.deadband']
        for arm in ['right', 'left']:
            for typ in ['pos', 'rot']:
                for axis in ['x', 'y', 'z']:
                    for gain in ['p', 'i', 'd']:
                        param_names.append(f'pid.{arm}.{typ}.{axis}.{gain}')

        future = self.param_client.get_parameters(param_names)
        future.add_done_callback(lambda fut: self._param_fetch_done(fut, param_names))
        self.params_need_fetch = False 

    def _param_fetch_done(self, future, param_names):
        """Callback when parameter data arrives."""
        try:
            results = future.result().values
            for i, name in enumerate(param_names):
                val = results[i]
                if val.type == 0: continue 
                
                extracted_val = 0.0
                if val.type == 1: extracted_val = float(val.bool_value)
                elif val.type == 2: extracted_val = float(val.integer_value)
                elif val.type == 3: extracted_val = val.double_value

                if name == 'pid.right.pos.deadband':
                    self.position_deadband = extracted_val
                else:
                    self.pid_gains[name] = extracted_val
            
            self.get_logger().info(f"Parameters updated. Deadband: {self.position_deadband}")
            self.visuals_need_update = True 
        except Exception as e:
            self.get_logger().error(f"Failed to process parameters: {e}")

def main():
    rclpy.init()
    node = HybridVisualizer()

    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Real-Time Cartesian Error & Dynamic Parameters', fontsize=16)
    
    lines = []
    deadband_spans = []
    pid_text_box = None

    colors = ['r', 'g', 'b']
    labels = ['X', 'Y', 'Z']

    for i, ax in enumerate(axes):
        ax.set_ylabel(f'{labels[i]} Error (m)')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='black', alpha=0.5)
        
        line, = ax.plot([], [], color=colors[i], linewidth=1.5, label=labels[i])
        lines.append(line)
        
        span = ax.axhspan(-0.005, 0.005, facecolor='green', alpha=0.2)
        deadband_spans.append(span)
        
        ax.legend(loc='upper right')

    axes[2].set_xlabel('Time (s)')
    plt.subplots_adjust(bottom=0.15) 

    def generate_pid_string():
        """Format the PID dictionary into a readable string."""
        text = ''
        for arm in ['right', 'left']:
            for typ in ['pos']: 
                text += f'{arm.upper()} {typ}: '
                for axis in ['x', 'y', 'z']:
                    p = node.pid_gains.get(f'pid.{arm}.{typ}.{axis}.p', 0)
                    i = node.pid_gains.get(f'pid.{arm}.{typ}.{axis}.i', 0)
                    d = node.pid_gains.get(f'pid.{arm}.{typ}.{axis}.d', 0)
                    text += f" {axis.upper()}[{p:.2f}|{i:.2f}|{d:.2f}]"
                text += '\n'
        return text

    def update_plot(frame):
        rclpy.spin_once(node, timeout_sec=0)

        if node.params_need_fetch:
            node.fetch_parameters_async()

        nonlocal pid_text_box
        if node.visuals_need_update:
            db = node.position_deadband
            
            for span in deadband_spans:
                try: span.remove()
                except: pass
            
            deadband_spans.clear()
            for ax in axes:
                s = ax.axhspan(-db, db, facecolor='green', alpha=0.2, label=f'Deadband (+/- {db})')
                deadband_spans.append(s)
            
            if pid_text_box: pid_text_box.remove()
            pid_str = generate_pid_string()
            pid_text_box = fig.text(0.5, 0.01, pid_str, fontsize=9, ha='center', va='bottom',
                                  bbox=dict(facecolor='white', alpha=0.9, boxstyle='round'))
            
            node.visuals_need_update = False

        if len(node.right_time) > 0:
            t = list(node.right_time)
            lines[0].set_data(t, list(node.right_linear_x))
            lines[1].set_data(t, list(node.right_linear_y))
            lines[2].set_data(t, list(node.right_linear_z))

            for ax in axes:
                ax.relim()
                ax.autoscale_view()

        return lines + deadband_spans

    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()