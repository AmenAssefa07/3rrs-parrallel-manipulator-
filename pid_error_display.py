#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

# PID gains
Kp, Ki, Kd = 0.013, 0.0, 0.0045  # Tuned PID gains
MAX_ANGLE, INTEGRAL_CLAMP = 6.0, 10.0  # Clamp values

# Derivative filter coefficient (0 < alpha < 1)
DERIVATIVE_FILTER_ALPHA = 0.1

# Enable or disable plotting
ENABLE_PLOTTING = True

# PID state variables
prev_error_x = prev_error_y = 0.0
integral_x = integral_y = 0.0
prev_time = time.time()
prev_derivative_x = prev_derivative_y = 0.0

# Default user input if nothing is published yet(target position and height)
desired_x, desired_y, desired_z = 500.0, 500.0, 100.0

# Track the last time ball position was updated
last_update_time = time.time()

# Data for plotting
error_x_list = []
error_y_list = []
magnitude_list = []
alpha_list = []
beta_list = []
time_list = []

def clamp(value, min_value, max_value):
    """Clamp a value within the specified range."""
    return max(min(value, max_value), min_value)

def pid_control(current_x, current_y):
    """Compute the pitch (α) and roll (β) using PID control."""
    global prev_error_x, prev_error_y, integral_x, integral_y, prev_time
    global prev_derivative_x, prev_derivative_y

    # Compute time delta
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    if dt <= 0:
        rospy.logwarn("Time delta is zero or negative, skipping this cycle.")
        return 0.0, 0.0

    # Calculate the proportional errors
    error_x = (desired_x - current_x)
    error_y = (desired_y - current_y)

    # Update integral terms with clamping
    integral_x = clamp(integral_x + error_x * dt, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
    integral_y = clamp(integral_y + error_y * dt, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)

    # Compute the derivative terms
    derivative_x = (error_x - prev_error_x) / dt
    derivative_y = (error_y - prev_error_y) / dt

    # Apply a low-pass filter to the derivatives
    filtered_derivative_x = (1 - DERIVATIVE_FILTER_ALPHA) * prev_derivative_x + DERIVATIVE_FILTER_ALPHA * derivative_x
    filtered_derivative_y = (1 - DERIVATIVE_FILTER_ALPHA) * prev_derivative_y + DERIVATIVE_FILTER_ALPHA * derivative_y

    # Update previous derivative values
    prev_derivative_x = filtered_derivative_x
    prev_derivative_y = filtered_derivative_y

    # Calculate the PID outputs
    alpha = Kp * error_x + Ki * integral_x + Kd * filtered_derivative_x
    beta = Kp * error_y + Ki * integral_y + Kd * filtered_derivative_y

    # Clamp the outputs to ±MAX_ANGLE degrees
    alpha = clamp(alpha, -MAX_ANGLE, MAX_ANGLE)
    beta = clamp(beta, -MAX_ANGLE, MAX_ANGLE)

    # Update the previous errors
    prev_error_x = error_x
    prev_error_y = error_y

    # Add errors and outputs to lists for plotting
    error_x_list.append(error_x)
    error_y_list.append(error_y)
    magnitude_list.append(math.sqrt(error_x**2 + error_y**2))
    alpha_list.append(alpha)
    beta_list.append(beta)
    time_list.append(current_time)

    return alpha, beta

def ball_position_callback(msg):
    """Callback to handle ball position updates."""
    global last_update_time

    # Update the last time a ball position was detected
    last_update_time = time.time()

    # Compute pitch (α) and roll (β)
    alpha, beta = pid_control(msg.x, msg.y)

    # Publish the desired orientation to the motor node
    orientation_msg = Float32MultiArray(data=[alpha, beta, desired_z])
    orientation_pub.publish(orientation_msg)

def user_input_callback(msg):
    """Callback to handle user input for target position and height."""
    global desired_x, desired_y, desired_z
    desired_x, desired_y, desired_z = msg.data

def publish_default_orientation(event):
    """Publish the default orientation if no ball is detected for 5 seconds."""
    if time.time() - last_update_time > 5.0:
        rospy.loginfo("No ball detected for 5 seconds. Moving ball around.")
        default_msg = Float32MultiArray(data=[2.0, 2.0, 100.0])
        orientation_pub.publish(default_msg)
        default_msg = Float32MultiArray(data=[-2.0, -2.0, 100.0])
        orientation_pub.publish(default_msg)

def pid_control_node():
    """Initialize the PID control ROS node."""
    rospy.init_node('pid_controller_node', anonymous=True)

    # Initialize publisher and subscribers
    global orientation_pub
    orientation_pub = rospy.Publisher('/desired_orientation', Float32MultiArray, queue_size=1)
    rospy.Subscriber('/ball_position', Vector3, ball_position_callback)
    rospy.Subscriber('/user_input', Float32MultiArray, user_input_callback)

    # Set a timer to publish default orientation periodically
    rospy.Timer(rospy.Duration(1), publish_default_orientation)

    rospy.loginfo("PID control node started.")

    if ENABLE_PLOTTING:
        # Start Matplotlib animation
        fig, axes = plt.subplots(4, 1, figsize=(10, 12))

        def update(frame):
            if len(time_list) > 0:
                for ax in axes:
                    ax.clear()

                # Plot error_x
                axes[0].plot(time_list, error_x_list, label='Error X')
                axes[0].set_ylabel('Error X')
                axes[0].legend()

                # Plot error_y
                axes[1].plot(time_list, error_y_list, label='Error Y')
                axes[1].set_ylabel('Error Y')
                axes[1].legend()

                # Plot magnitude
                axes[2].plot(time_list, magnitude_list, label='Magnitude')
                axes[2].set_ylabel('Magnitude')
                axes[2].legend()

                # Plot alpha and beta
                axes[3].plot(time_list, alpha_list, label='Alpha')
                axes[3].plot(time_list, beta_list, label='Beta')
                axes[3].set_ylabel('Alpha/Beta')
                axes[3].legend()

                axes[3].set_xlabel('Time')

        ani = FuncAnimation(fig, update, interval=100)
        plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        pid_control_node()
    except rospy.ROSInterruptException:
        pass
