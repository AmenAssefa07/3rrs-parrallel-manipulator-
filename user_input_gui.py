#!/usr/bin/env python3

import tkinter as tk
import subprocess
import os
import signal
import rospy
from std_msgs.msg import Float32MultiArray
from threading import Thread
import time

class UserInputGUI:
    def __init__(self, root):
        # Initialize ROS node
        rospy.init_node('user_input_gui', anonymous=True)

        # ROS Publisher for user input
        self.input_pub = rospy.Publisher('/user_input', Float32MultiArray, queue_size=1)

        # Store process handlers for nodes
        self.processes = {}

        # Create GUI elements
        root.title("3-RRS Platform Control GUI")

        # Input fields for X, Y, Z values
        tk.Label(root, text="Desired X:").grid(row=0, column=0)
        self.x_entry = tk.Entry(root)
        self.x_entry.grid(row=0, column=1)

        tk.Label(root, text="Desired Y:").grid(row=1, column=0)
        self.y_entry = tk.Entry(root)
        self.y_entry.grid(row=1, column=1)

        tk.Label(root, text="Desired Z:").grid(row=2, column=0)
        self.z_entry = tk.Entry(root)
        self.z_entry.grid(row=2, column=1)

        # Flag to control the publishing thread
        self.publish_flag = False

        # Button to start/stop constant publishing
        self.toggle_button = tk.Button(root, text="Start Publishing", command=self.toggle_publishing)
        self.toggle_button.grid(row=3, column=0, columnspan=2)

        # Start/Stop buttons for each node
        self.create_node_controls(root, "Ball Tracker", "/home/rrspi/ros_catkin_ws/src/ball_tracker/src/ball_tracker.py", row=4)
        self.create_node_controls(root, "PID Controller", "/home/rrspi/ros_catkin_ws/src/motor_control/src/pid_controller_node.py", row=5)
        self.create_node_controls(root, "Motor Control", "/home/rrspi/ros_catkin_ws/src/motor_control/src/motor_control_node.py", row=6)

    def create_node_controls(self, root, label, file_path, row):
        """Create Start/Stop buttons for each node."""
        tk.Label(root, text=label).grid(row=row, column=0)

        start_button = tk.Button(root, text="Start", command=lambda: self.start_node(file_path))
        start_button.grid(row=row, column=1)

        stop_button = tk.Button(root, text="Stop", command=lambda: self.stop_node(file_path))
        stop_button.grid(row=row, column=2)

    def start_node(self, file_path):
        """Start the given ROS node as a new process group."""
        if file_path not in self.processes:
            cmd = f"python3 {file_path}"
            process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            self.processes[file_path] = process
            rospy.loginfo(f"{file_path} started with PID {process.pid}.")

    def stop_node(self, file_path):
        """Stop the given ROS node by killing its process group."""
        if file_path in self.processes:
            process = self.processes[file_path]
            try:
                # Send SIGTERM to the entire process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait()  # Wait for the process to terminate
                rospy.loginfo(f"{file_path} stopped successfully.")
            except Exception as e:
                rospy.logerr(f"Failed to stop {file_path}: {e}")
            finally:
                # Ensure the process is removed from tracking
                del self.processes[file_path]
        else:
            rospy.logwarn(f"{file_path} was not running.")

    def toggle_publishing(self):
        """Start or stop the publishing loop."""
        if not self.publish_flag:
            self.publish_flag = True
            self.toggle_button.config(text="Stop Publishing")
            self.publisher_thread = Thread(target=self.publish_loop)
            self.publisher_thread.daemon = True
            self.publisher_thread.start()
        else:
            self.publish_flag = False
            self.toggle_button.config(text="Start Publishing")

    def publish_loop(self):
        """Continuously publish the desired XYZ values."""
        while self.publish_flag:
            try:
                x = float(self.x_entry.get())
                y = float(self.y_entry.get())
                z = float(self.z_entry.get())

                msg = Float32MultiArray(data=[x, y, z])
                self.input_pub.publish(msg)
                #rospy.loginfo(f"Published: X={x}, Y={y}, Z={z}")
            except ValueError:
                rospy.logerr("Invalid input. Please enter numeric values.")
            time.sleep(0.1)  # Adjust the rate of publishing as needed

    def on_close(self):
        """Ensure all nodes are stopped when the GUI is closed."""
        self.publish_flag = False  # Stop the publishing loop
        for file_path in list(self.processes.keys()):
            self.stop_node(file_path)
        rospy.loginfo("All nodes stopped. Exiting.")
        root.destroy()

if __name__ == '__main__':
    try:
        root = tk.Tk()
        gui = UserInputGUI(root)
        root.protocol("WM_DELETE_WINDOW", gui.on_close)  # Handle window close
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
