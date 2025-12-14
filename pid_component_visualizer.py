"""
PID Component Visualizer

Subscribes to a specific PID component topic (e.g., /right_arm/pid_components/pos_z)
and plots the P, I, D, and Total contributions in real-time.  
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import argparse

class PIDComponentVisualizer(Node):
    def __init__(self, arm_side, axis, max_points=500):
        super().__init__('pid_component_visualizer')
        
        if arm_side not in ['right', 'left']:
            raise ValueError("Arm side must be 'right' or 'left'")
        if axis not in ['pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z']:
            raise ValueError("Axis must be one of pos_x, pos_y, pos_z, rot_x, rot_y, rot_z")

        self.max_points = max_points
        self.time_data = deque(maxlen=max_points)
        self.p_data = deque(maxlen=max_points)
        self.i_data = deque(maxlen=max_points)
        self.d_data = deque(maxlen=max_points)
        self.total_data = deque(maxlen=max_points)
        
        self.start_time = self.get_clock().now()

        # Construct the topic name from arguments
        self.topic_name = f'/{arm_side}_arm/pid_components/{axis}'

        # Subscribe to the specified topic
        self.sub = self.create_subscription(
            TwistStamped,
            self.topic_name,
            self.data_callback,
            10)
        self.get_logger().info(f"PID Component Visualizer started for topic: '{self.topic_name}'")

    def data_callback(self, msg: TwistStamped):
        """Callback to store the received PID component data."""
        elapsed_time = (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9) - \
                       (self.start_time.nanoseconds / 1e9)
        self.time_data.append(elapsed_time)

        # Unpack the data from the TwistStamped message based on the mapping
        self.p_data.append(msg.twist.linear.x)      # P term
        self.i_data.append(msg.twist.linear.y)      # I term
        self.d_data.append(msg.twist.linear.z)      # D term
        self.total_data.append(msg.twist.angular.x) # Total

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Visualize PID components for a specific arm and axis.')
    parser.add_argument(
        '--arm', 
        type=str,
        required=True,
        choices=['right', 'left'],
        help="The arm to visualize ('right' or 'left')."
    )
    parser.add_argument(
        '--axis',
        type=str,
        required=True,
        choices=['pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z'],
        help="The specific axis controller to visualize (e.g., 'pos_z')."
    )

    ros_args_index = -1
    for i, arg in enumerate(sys.argv):
        if arg == '--ros-args':
            ros_args_index = i
            break
    if ros_args_index != -1:
        cli_args = sys.argv[1:ros_args_index]
        ros_args = sys.argv[ros_args_index:]
    else:
        cli_args = sys.argv[1:]
        ros_args = None

    args = parser.parse_args(cli_args)

    rclpy.init(args=ros_args)
    visualizer_node = PIDComponentVisualizer(arm_side=args.arm, axis=args.axis)

    fig, axes = plt.subplots(4, 1, figsize=(15, 12), sharex=True)
    title = f"PID Component Analysis: {args.arm.capitalize()} Arm - {args.axis.replace('_', ' ').upper()}"
    fig.suptitle(title, fontsize=16)

    ax_p, ax_i, ax_d, ax_total = axes

    # Setup each subplot individually
    ax_p.set_title('P (Proportional) Term')
    ax_p.set_ylabel('Force/Torque')
    ax_p.grid(True, linestyle='--')
    line_p, = ax_p.plot([], [], color='blue', label='P Term')
    ax_p.legend()

    ax_i.set_title('I (Integral) Term')
    ax_i.set_ylabel('Force/Torque')
    ax_i.grid(True, linestyle='--')
    line_i, = ax_i.plot([], [], color='red', label='I Term')
    ax_i.legend()

    ax_d.set_title('D (Derivative) Term')
    ax_d.set_ylabel('Force/Torque')
    ax_d.grid(True, linestyle='--')
    line_d, = ax_d.plot([], [], color='green', label='D Term')
    ax_d.legend()

    ax_total.set_title('Total Output')
    ax_total.set_xlabel('Time (s)')
    ax_total.set_ylabel('Total Force/Torque')
    ax_total.grid(True, linestyle='--')
    line_total, = ax_total.plot([], [], color='black', linewidth=2, label='Total')
    ax_total.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])

    def update_plot(frame):
        rclpy.spin_once(visualizer_node, timeout_sec=0)
        
        if len(visualizer_node.time_data) > 0:
            times = list(visualizer_node.time_data)
            
            # Update data for each line
            line_p.set_data(times, list(visualizer_node.p_data))
            line_i.set_data(times, list(visualizer_node.i_data))
            line_d.set_data(times, list(visualizer_node.d_data))
            line_total.set_data(times, list(visualizer_node.total_data))
            
            # Autoscale all subplots
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
                
        return line_p, line_i, line_d, line_total

    ani = FuncAnimation(fig, update_plot, interval=50, blit=True, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main()