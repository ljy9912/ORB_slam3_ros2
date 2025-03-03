#!/usr/bin/env python3
"""
Improved Multi-Joint Calibration Script

This script performs calibration for multiple anatomical points (wrist and shoulder)
using ROS 2 nodes that subscribe directly to tracker topics.

Supports two modes:
1. Bag playback (when bag files are specified)
2. Real-time data capture with interactive controls (default)

Mathematical Foundation:
----------------------
For each fixed point (wrist/shoulder), the rigid body calibration is based on:
    p_tracker = R_tracker * p_fixed_tracker + p_fixed

Where for each joint:
    - p_tracker: Position of the tracker in world coordinates (measured)
    - R_tracker: Rotation matrix of the tracker (measured)
    - p_fixed_tracker: Position of the fixed point in tracker coordinates (unknown)
    - p_fixed: Position of the fixed point in world coordinates (unknown)

The system is solved using least squares to minimize the residual error.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import os
import time
import threading
import sys
from typing import Dict, List, Tuple, Optional


class JointCalibratorNode(Node):
    """
    ROS 2 node for calibrating joint positions from tracker poses.
    """
    
    def __init__(self, joint_name: str, topic_name: str, update_interval: int = 100):
        """
        Initialize the joint calibrator node.
        
        Args:
            joint_name: Name of the joint being calibrated
            topic_name: ROS topic for tracker poses
            update_interval: Samples between status updates
        """
        super().__init__(f"{joint_name}_calibrator_node")
        
        # Store configuration
        self.joint_name = joint_name
        self.topic_name = topic_name
        self.update_interval = update_interval
        
        # Initialize data storage
        self.coefficient_matrix = np.zeros((0, 6))
        self.dependent_vector = np.zeros((0))
        self.sample_count = 0
        self.solution = None
        self.error = None
        
        # Flag to control data collection
        self.collecting_data = False
        
        # Create subscription to tracker topic
        self.subscription = self.create_subscription(
            Pose, self.topic_name, self.pose_callback, 10
        )
        
        self.get_logger().info(
            f"Calibrating {self.joint_name} from topic {self.topic_name}"
        )
    
    def start_collection(self):
        """Start collecting data samples"""
        self.collecting_data = True
        self.get_logger().info(f"Starting data collection for {self.joint_name}")
        
    def stop_collection(self):
        """Stop collecting data samples"""
        self.collecting_data = False
        self.get_logger().info(
            f"Stopped data collection for {self.joint_name} with {self.sample_count} samples"
        )
    
    def pose_callback(self, msg: Pose) -> None:
        """
        Process incoming pose messages to build the calibration matrices.
        
        Args:
            msg: The pose message containing position and orientation data
        """
        # Only process data when collection is active
        if not self.collecting_data:
            return
            
        # Extract position and orientation from the pose message
        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        quaternion = np.array([
            msg.orientation.x, 
            msg.orientation.y, 
            msg.orientation.z, 
            msg.orientation.w
        ])
        
        # Apply coordinate system correction if needed (invert Z component)
        quaternion[2] *= -1
        
        # Convert quaternion to rotation matrix
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        
        # Construct the augmented matrix for this sample
        # [R | -I] where R is rotation matrix and I is identity
        new_row = np.concatenate((rotation_matrix, -np.eye(3)), axis=1)
        
        # Append to our system of equations
        self.coefficient_matrix = np.concatenate((self.coefficient_matrix, new_row), axis=0)
        self.dependent_vector = np.concatenate((self.dependent_vector, -position))
        
        # Update sample count and report progress
        self.sample_count += 1
        if self.sample_count % self.update_interval == 0:
            self.get_logger().info(f"Collected {self.sample_count} samples for {self.joint_name}")
    
    def compute_calibration(self) -> Tuple[Optional[np.ndarray], Optional[float]]:
        """
        Solve the least squares problem to find the calibration parameters.
        
        Returns:
            Tuple containing the solution vector and normalized error
        """
        if self.sample_count == 0:
            self.get_logger().error(f"No samples available for {self.joint_name} calibration")
            return None, None
        
        self.get_logger().info(
            f"Computing {self.joint_name} calibration with {self.sample_count} samples"
        )
        
        try:
            # Solve the least squares problem
            solution, residuals, rank, singular_values = np.linalg.lstsq(
                self.coefficient_matrix, self.dependent_vector, rcond=None
            )
            
            # Calculate normalized error
            normalized_error = residuals[0] / self.sample_count if len(residuals) > 0 else 0
            
            # Store results
            self.solution = solution
            self.error = normalized_error
            
            # Split solution into components for clarity
            joint_in_tracker = solution[0:3]
            joint_in_world = solution[3:6]
            
            # Log detailed results
            self.get_logger().info(
                f"{self.joint_name.upper()} CALIBRATION RESULTS:\n"
                f"  Position in tracker coordinates: [{joint_in_tracker[0]:.4f}, {joint_in_tracker[1]:.4f}, {joint_in_tracker[2]:.4f}]\n"
                f"  Position in world coordinates: [{joint_in_world[0]:.4f}, {joint_in_world[1]:.4f}, {joint_in_world[2]:.4f}]\n"
                f"  Normalized error: {normalized_error:.6f}"
            )
            
            return solution, normalized_error
            
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Linear algebra error during {self.joint_name} calibration: {e}")
            return None, None
    
    def save_calibration(self, file_path: str) -> bool:
        """
        Save the calibration results to a file.
        
        Args:
            file_path: Path where calibration should be saved
            
        Returns:
            True if successful, False otherwise
        """
        if self.solution is None:
            self.get_logger().error(f"No calibration results available for {self.joint_name}")
            return False
        
        try:
            # Ensure output directory exists
            os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
            
            # Save calibration results
            np.savez(
                file_path,
                joint_name=self.joint_name,
                joint_in_tracker=self.solution[0:3],
                joint_in_world=self.solution[3:6],
                error=self.error,
                sample_count=self.sample_count
            )
            
            self.get_logger().info(f"Saved {self.joint_name} calibration to {file_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving {self.joint_name} calibration: {e}")
            return False


def run_bag_calibration(bag_file: str, topic: str, joint_name: str, duration: float = 30.0) -> Tuple[Optional[np.ndarray], Optional[float]]:
    """
    Run calibration by playing a bag file and subscribing to the tracker topic.
    
    Args:
        bag_file: Path to the ROS 2 bag file
        topic: Topic name to subscribe to
        joint_name: Name of the joint being calibrated
        duration: Maximum duration to process the bag (seconds)
        
    Returns:
        Tuple containing solution and error, or (None, None) if failed
    """
    import subprocess
    
    # Start the bag playback in a separate process
    try:
        bag_process = subprocess.Popen(
            ["ros2", "bag", "play", bag_file, "--topics", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
    except Exception as e:
        print(f"Error starting bag playback: {e}")
        return None, None
    
    # Initialize ROS node
    rclpy.init(args=None)
    node = JointCalibratorNode(joint_name, topic)
    
    # Start data collection immediately
    node.start_collection()
    
    # Function to spin the node in a separate thread
    def spin_node():
        rclpy.spin(node)
    
    # Start the node spinning
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.daemon = True
    spin_thread.start()
    
    # Wait for the specified duration or until the bag finishes
    print(f"Processing {joint_name} calibration from {bag_file}...")
    try:
        start_time = time.time()
        while (time.time() - start_time < duration and 
               bag_process.poll() is None):
            # Check if we have enough samples
            if node.sample_count > 0 and node.sample_count % 10 == 0:
                print(f"Collected {node.sample_count} samples for {joint_name}")
            time.sleep(0.1)
        
        # Stop data collection
        node.stop_collection()
        
        # Compute final calibration
        solution, error = node.compute_calibration()
        
        # Save the calibration if successful
        if solution is not None:
            output_dir = f"./calibration_results"
            os.makedirs(output_dir, exist_ok=True)
            node.save_calibration(f"{output_dir}/{joint_name}_calibration.npz")
        
        return solution, error
        
    except KeyboardInterrupt:
        print("Calibration interrupted by user")
    finally:
        # Clean up
        if bag_process.poll() is None:
            bag_process.terminate()
        rclpy.shutdown()
    
    return None, None


def run_interactive_calibration(joint_config: Dict[str, str]) -> Dict[str, Tuple[np.ndarray, float]]:
    """
    Run calibration in interactive mode for all joints.
    
    Args:
        joint_config: Dictionary mapping joint names to topic names
        
    Returns:
        Dictionary of joint name to (solution, error) tuples
    """
    # Initialize ROS 
    rclpy.init(args=None)
    
    # Create nodes for each joint
    nodes = {}
    for joint_name, topic in joint_config.items():
        nodes[joint_name] = JointCalibratorNode(joint_name, topic)
    
    # Function to spin the nodes in a separate thread
    def spin_nodes():
        try:
            executor = rclpy.executors.MultiThreadedExecutor()
            for node in nodes.values():
                executor.add_node(node)
            executor.spin()
        except Exception as e:
            print(f"Error in executor: {e}")
    
    # Start the nodes spinning
    spin_thread = threading.Thread(target=spin_nodes)
    spin_thread.daemon = True
    spin_thread.start()
    
    results = {}
    
    # Interactive calibration process
    try:
        print("\n" + "="*80)
        print("INTERACTIVE JOINT CALIBRATION")
        print("="*80)
        print("\nPress Enter to begin calibration process...")
        input()
        
        # Process each joint sequentially
        for joint_name, node in nodes.items():
            print(f"\n{joint_name.upper()} CALIBRATION:")
            print(f"Make sure the {joint_name} tracker is visible and positioned correctly.")
            print("Press Enter to START collecting data...")
            input()
            
            # Start data collection
            node.start_collection()
            
            print(f"Collecting data for {joint_name}... Move the tracker around while keeping the")
            print(f"anatomical point fixed. Try to cover different orientations.")
            print("Press Enter to STOP collecting data...")
            input()
            
            # Stop data collection
            node.stop_collection()
            
            # Compute calibration
            solution, error = node.compute_calibration()
            results[joint_name] = (solution, error)
            
            # Save individual calibration
            if solution is not None:
                output_dir = f"./calibration_results"
                os.makedirs(output_dir, exist_ok=True)
                node.save_calibration(f"{output_dir}/{joint_name}_calibration.npz")
        
        # Save combined results if all calibrations succeeded
        if all(solution is not None for solution, _ in results.values()):
            output_dir = "./calibration_results"
            os.makedirs(output_dir, exist_ok=True)
            combined_data = {}
            
            for joint_name, (solution, error) in results.items():
                combined_data[f"{joint_name}_tracker"] = solution[0:3]
                combined_data[f"{joint_name}_world"] = solution[3:6]
                combined_data[f"{joint_name}_error"] = error
            
            np.savez(f"{output_dir}/combined_calibration.npz", **combined_data)
            print(f"\nCombined calibration saved to {output_dir}/combined_calibration.npz")
        
        return results
        
    except KeyboardInterrupt:
        print("\nCalibration process interrupted by user.")
        return {name: (None, None) for name in nodes.keys()}
    finally:
        # Clean up
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()


def print_calibration_results(joint_results: Dict[str, Tuple[np.ndarray, float]]):
    """
    Print a summary of calibration results for all joints.
    
    Args:
        joint_results: Dictionary of joint name to (solution, error) tuples
    """
    print("\n" + "="*80)
    print("CALIBRATION SUMMARY")
    print("="*80)
    
    for joint_name, (solution, error) in joint_results.items():
        if solution is not None:
            joint_in_tracker = solution[0:3]
            joint_in_world = solution[3:6]
            
            print(f"\n{joint_name.upper()}:")
            print(f"  Position in tracker coordinates: [{joint_in_tracker[0]:.4f}, {joint_in_tracker[1]:.4f}, {joint_in_tracker[2]:.4f}]")
            print(f"  Position in world coordinates: [{joint_in_world[0]:.4f}, {joint_in_world[1]:.4f}, {joint_in_world[2]:.4f}]")
            print(f"  Normalized error: {error:.6f}")
        else:
            print(f"\n{joint_name.upper()}: Calibration failed")
    
    print("\n" + "="*80)


def main():
    """
    Main entry point for the multi-joint calibration script.
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Multi-joint calibration with interactive mode")
    parser.add_argument("--wrist-bag", help="Bag file for wrist calibration (optional)")
    parser.add_argument("--wrist-topic", default="/manus_tracker_right", help="Topic for wrist tracker")
    parser.add_argument("--shoulder-bag", help="Bag file for shoulder calibration (optional)")
    parser.add_argument("--shoulder-topic", default="/elbow_tracker", help="Topic for shoulder tracker")
    parser.add_argument("--duration", type=float, default=30.0, help="Maximum duration for processing each bag (if bag mode)")
    args = parser.parse_args()
    
    # Determine whether to use bag files or interactive mode
    use_bag_mode = args.wrist_bag is not None and args.shoulder_bag is not None
    
    joint_results = {}
    
    if use_bag_mode:
        print("Starting joint calibration using bag files...")
        
        # Calibrate wrist
        print("\n" + "="*80)
        print(f"CALIBRATING WRIST")
        print("="*80)
        solution, error = run_bag_calibration(
            args.wrist_bag,
            args.wrist_topic,
            "wrist",
            args.duration
        )
        joint_results["wrist"] = (solution, error)
        
        # Calibrate shoulder
        print("\n" + "="*80)
        print(f"CALIBRATING SHOULDER")
        print("="*80)
        solution, error = run_bag_calibration(
            args.shoulder_bag,
            args.shoulder_topic,
            "shoulder",
            args.duration
        )
        joint_results["shoulder"] = (solution, error)
    else:
        # Interactive real-time calibration mode
        joint_config = {
            "wrist": args.wrist_topic,
            "shoulder": args.shoulder_topic
        }
        joint_results = run_interactive_calibration(joint_config)
    
    # Print summary of all calibrations
    print_calibration_results(joint_results)
    
    return 0


if __name__ == "__main__":
    main()