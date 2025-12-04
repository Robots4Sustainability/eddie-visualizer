#!/usr/bin/env python3
"""
Demo script that publishes realistic cartesian error variations.
This simulates the behavior of the eddie_ros interface during arm movement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import random


class CartesianErrorDemoPublisher(Node):
    def __init__(self):
        #super().__init__('cartesian_error_demo_publisher')
        super().__init__('eddie_ros_interface')
        
        self.get_logger().info("Declaring PID and deadband parameters...")
        self.declare_parameter("pid.right.pos.deadband", 0.03)
        self.declare_parameter("pid.right.rot.deadband", 0.03)
        # Declare fake PID gain parameters for both arms
        for arm in ['right', 'left']:
            for typ in ['pos', 'rot']:
                for axis in ['x', 'y', 'z']:
                    for gain, val in zip(['p', 'i', 'd'], [1.0, 0.1, 0.01]):
                        param_name = f"pid.{arm}.{typ}.{axis}.{gain}"
                        self.declare_parameter(param_name, val + random.uniform(-0.2, 0.2))

        pos_db = self.get_parameter('pid.right.pos.deadband').get_parameter_value().double_value
        self.get_logger().info(f"Deadband parameter 'pid.right.pos.deadband' set to: {pos_db}")
        
        # Publishers for both arms
        self.right_pub = self.create_publisher(
            Twist,
            'right_arm/cartesian_error',
            10
        )
        self.left_pub = self.create_publisher(
            Twist,
            'left_arm/cartesian_error',
            10
        )
        
        # Timer to publish at 50 Hz (matching the interface's 20ms timer)
        self.timer = self.create_timer(0.02, self.publish_errors)
        
        # Simulation state
        self.time = 0.0
        self.dt = 0.02  # 20ms
        
        # Scenario state machine
        self.scenario = 'converging'
        self.scenario_time = 0.0
        self.scenario_duration = 5.0  # Each scenario lasts 5 seconds
        
        # Initial error values (in meters and radians)
        self.right_error = {
            'linear': [0.0, 0.0, 0.0],
            'angular': [0.0, 0.0, 0.0]
        }
        self.left_error = {
            'linear': [0.0, 0.0, 0.0],
            'angular': [0.0, 0.0, 0.0]
        }
        
        self.get_logger().info('Cartesian Error Demo Publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - right_arm/cartesian_error')
        self.get_logger().info('  - left_arm/cartesian_error')
        self.get_logger().info('')
        self.get_logger().info('Scenarios will cycle every 5 seconds:')
        self.get_logger().info('  1. Converging to target (errors decrease)')
        self.get_logger().info('  2. Step response (sudden change, then settle)')
        self.get_logger().info('  3. Oscillatory behavior (underdamped)')
        self.get_logger().info('  4. Tracking (small continuous errors)')
        self.get_logger().info('  5. Large movement (bigger errors)')
    
    def converging_scenario(self):
        """Simulate errors converging to zero (well-damped response)"""
        # Exponential decay with some noise
        decay_rate = 0.15
        
        for arm_error in [self.right_error, self.left_error]:
            for i in range(3):
                # Linear errors decay from initial offset
                target = 0.0
                current = arm_error['linear'][i]
                arm_error['linear'][i] = current * (1 - decay_rate) + random.gauss(0, 0.001)
                
                # Angular errors decay from initial offset
                target = 0.0
                current = arm_error['angular'][i]
                arm_error['angular'][i] = current * (1 - decay_rate) + random.gauss(0, 0.01)
    
    def step_response_scenario(self):
        """Simulate a step response with some overshoot"""
        if self.scenario_time < 0.1:
            # Initial step
            self.right_error['linear'] = [0.08, -0.05, 0.06]
            self.right_error['angular'] = [0.3, -0.2, 0.25]
            self.left_error['linear'] = [-0.06, 0.07, -0.04]
            self.left_error['angular'] = [-0.25, 0.3, -0.15]
        else:
            # Converge with slight overshoot
            decay_rate = 0.12
            overshoot_freq = 3.0  # Hz
            overshoot_amp = 0.015
            
            overshoot = overshoot_amp * math.sin(2 * math.pi * overshoot_freq * self.scenario_time) * \
                       math.exp(-self.scenario_time * 2)
            
            for arm_error in [self.right_error, self.left_error]:
                for i in range(3):
                    arm_error['linear'][i] = arm_error['linear'][i] * (1 - decay_rate) + \
                                            overshoot + random.gauss(0, 0.0008)
                    arm_error['angular'][i] = arm_error['angular'][i] * (1 - decay_rate) + \
                                             overshoot * 2 + random.gauss(0, 0.008)
    
    def oscillatory_scenario(self):
        """Simulate underdamped oscillatory behavior"""
        freq = 2.0  # Hz
        decay = 0.5
        
        # Oscillating errors
        for i, arm_error in enumerate([self.right_error, self.left_error]):
            phase_offset = i * math.pi / 4  # Different phase for each arm
            
            for j in range(3):
                amplitude_linear = 0.04 * math.exp(-decay * self.scenario_time)
                amplitude_angular = 0.2 * math.exp(-decay * self.scenario_time)
                
                axis_freq = freq * (1.0 + j * 0.3)  # Different frequencies per axis
                
                arm_error['linear'][j] = amplitude_linear * \
                    math.sin(2 * math.pi * axis_freq * self.scenario_time + phase_offset + j) + \
                    random.gauss(0, 0.001)
                    
                arm_error['angular'][j] = amplitude_angular * \
                    math.sin(2 * math.pi * axis_freq * self.scenario_time + phase_offset + j + 1) + \
                    random.gauss(0, 0.01)
    
    def tracking_scenario(self):
        """Simulate continuous tracking with small errors"""
        # Small sinusoidal variations representing tracking errors
        for i, arm_error in enumerate([self.right_error, self.left_error]):
            phase_offset = i * math.pi / 3
            
            for j in range(3):
                freq1 = 0.5 + j * 0.2
                freq2 = 1.5 + j * 0.3
                
                arm_error['linear'][j] = \
                    0.008 * math.sin(2 * math.pi * freq1 * self.time + phase_offset) + \
                    0.003 * math.sin(2 * math.pi * freq2 * self.time + phase_offset + 1) + \
                    random.gauss(0, 0.0005)
                
                arm_error['angular'][j] = \
                    0.05 * math.sin(2 * math.pi * freq1 * self.time + phase_offset + 2) + \
                    0.02 * math.sin(2 * math.pi * freq2 * self.time + phase_offset + 3) + \
                    random.gauss(0, 0.005)
    
    def large_movement_scenario(self):
        """Simulate larger movements with bigger initial errors"""
        if self.scenario_time < 0.1:
            # Large initial error
            self.right_error['linear'] = [0.15, -0.12, 0.10]
            self.right_error['angular'] = [0.5, -0.4, 0.45]
            self.left_error['linear'] = [-0.10, 0.13, -0.11]
            self.left_error['angular'] = [-0.42, 0.48, -0.38]
        else:
            # Faster convergence for large movements
            decay_rate = 0.18
            
            for arm_error in [self.right_error, self.left_error]:
                for i in range(3):
                    arm_error['linear'][i] = arm_error['linear'][i] * (1 - decay_rate) + \
                                            random.gauss(0, 0.002)
                    arm_error['angular'][i] = arm_error['angular'][i] * (1 - decay_rate) + \
                                             random.gauss(0, 0.015)
    
    def publish_errors(self):
        """Publish cartesian errors for both arms"""
        self.time += self.dt
        self.scenario_time += self.dt
        
        # Switch scenarios every 5 seconds
        if self.scenario_time >= self.scenario_duration:
            self.scenario_time = 0.0
            scenarios = ['converging', 'step_response', 'oscillatory', 'tracking', 'large_movement']
            current_idx = scenarios.index(self.scenario)
            next_idx = (current_idx + 1) % len(scenarios)
            self.scenario = scenarios[next_idx]
            self.get_logger().info(f'\n--- Switching to scenario: {self.scenario} ---\n')
        
        # Run the current scenario
        if self.scenario == 'converging':
            if self.scenario_time < 0.1:
                # Initialize with some error
                self.right_error['linear'] = [0.05, -0.03, 0.04]
                self.right_error['angular'] = [0.2, -0.15, 0.18]
                self.left_error['linear'] = [-0.04, 0.05, -0.03]
                self.left_error['angular'] = [-0.18, 0.2, -0.12]
            self.converging_scenario()
        elif self.scenario == 'step_response':
            self.step_response_scenario()
        elif self.scenario == 'oscillatory':
            self.oscillatory_scenario()
        elif self.scenario == 'tracking':
            self.tracking_scenario()
        elif self.scenario == 'large_movement':
            self.large_movement_scenario()
        
        # Publish right arm error
        right_msg = Twist()
        right_msg.linear.x = self.right_error['linear'][0]
        right_msg.linear.y = self.right_error['linear'][1]
        right_msg.linear.z = self.right_error['linear'][2]
        right_msg.angular.x = self.right_error['angular'][0]
        right_msg.angular.y = self.right_error['angular'][1]
        right_msg.angular.z = self.right_error['angular'][2]
        self.right_pub.publish(right_msg)
        
        # Publish left arm error
        left_msg = Twist()
        left_msg.linear.x = self.left_error['linear'][0]
        left_msg.linear.y = self.left_error['linear'][1]
        left_msg.linear.z = self.left_error['linear'][2]
        left_msg.angular.x = self.left_error['angular'][0]
        left_msg.angular.y = self.left_error['angular'][1]
        left_msg.angular.z = self.left_error['angular'][2]
        self.left_pub.publish(left_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianErrorDemoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
