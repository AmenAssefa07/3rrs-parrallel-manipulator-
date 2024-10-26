#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import threading
import time

# PID gains
Kp = 0.02  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.005  # Derivative gain

# Clamp values
MAX_ANGLE = 2.0  # Max pitch/roll in degrees
INTEGRAL_CLAMP = 2.0  # Prevent integral windup

# Derivative filter coefficient (0 < alpha < 1)
DERIVATIVE_FILTER_ALPHA = 0.1

# PID state variables
prev_error_x, prev_error_y = 0.0, 0.0
integral_x, integral_y = 0.0, 0.0
prev_time = time.time()
prev_derivative_x, prev_derivative_y = 0.0, 0.0

# Desired values from user input
desired_x, desired_y, desired_z = 0.0, 0.0, 100.0  # Default Z height

def clamp(value, min_value, max_value):
    """Clamp a value within a specified range."""
    return max(min(value, max_value), min_value)

def pid_control(current_x, current_y):
    """Compute the pitch (α) and roll (β) using PID control with integral clamping and derivative filtering."""
    global prev_error_x, prev_error_y, integral_x, integral_y, prev_time
    global prev_derivative_x, prev_derivative_y

    # Compute time delta
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    if dt <= 0:
        rospy.logwarn("Time delta is zero or negative, skipping this cycle.")
        return 0.0, 0.0

    # Calculate errors
    error_x = desired_x - current_x
    error_y = desired_y - current_y

    # Update integrals with clamping to prevent windup
    integral_x += error_x * dt
    integral_y += error_y * dt
    integral_x = clamp(integral_x, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
    integral_y = clamp(integral_y, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)

    # Compute unfiltered derivatives
    derivative_x = (error_x - prev_error_x) / dt
    derivative_y = (error_y - prev_error_y) / dt

    # Apply low-pass filter to the derivative
    filtered_derivative_x = (1 - DERIVATIVE_FILTER_ALPHA) * prev_derivative_x + DERIVATIVE_FILTER_ALPHA * derivative_x
    filtered_derivative_y = (1 - DERIVATIVE_FILTER_ALPHA) * prev_derivative_y + DERIVATIVE_FILTER_ALPHA * derivative_y

    # Update previous derivative values
    prev_derivative_x = filtered_derivative_x
    prev_derivative_y = filtered_derivative_y

    # PID calculations
    alpha = Kp * error_x + Ki * integral_x + Kd * filtered_derivative_x
    beta = Kp * error_y + Ki * integral_y + Kd * filtered_derivative_y

    # Clamp α and β to ±MAX_ANGLE degrees
    alpha = clamp(alpha, -MAX_ANGLE, MAX_ANGLE)
    beta = clamp(beta, -MAX_ANGLE, MAX_ANGLE)

    # Update previous errors
    prev_error_x = error_x
    prev_error_y = error_y

    return alpha, beta

def ball_position_callback(msg):
    """Callback to receive ball position and velocity."""
    current_x, current_y = msg.x, msg.y

    # Compute pitch (α) and roll (β) corrections
    alpha, beta = pid_control(current_x, current_y)

    # Publish the corrected orientation
    orientation_msg = Float32MultiArray()
    orientation_msg.data = [alpha, beta, desired_z]
    orientation_pub.publish(orientation_msg)

def user_input_callback(msg):
    """Callback to receive desired ball position and platform height."""
    global desired_x, desired_y, desired_z
    desired_x, desired_y, desired_z = msg.data

def pid_control_node():
    """Initialize the PID control ROS node."""
    rospy.init_node('pid_control_node', anonymous=True)

    # Subscribers
    rospy.Subscriber('/ball_position', Vector3, ball_position_callback)
    rospy.Subscriber('/user_input', Float32MultiArray, user_input_callback)

    # Publisher for desired orientation
    global orientation_pub
    orientation_pub = rospy.Publisher('/desired_orientation', Float32MultiArray, queue_size=10)

    rospy.loginfo("PID control node started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        pid_control_node()
    except rospy.ROSInterruptException:
        pass
