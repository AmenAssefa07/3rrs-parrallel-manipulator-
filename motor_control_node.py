#!/usr/bin/env python3

import rospy
import serial
import threading
import math
from std_msgs.msg import Float32MultiArray

# Initialize serial connection to Pololu Maestro
try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    rospy.loginfo("Connected to Pololu Maestro on /dev/serial0")
except serial.SerialException as e:
    rospy.logerr(f"Error: {e}")
    exit(1)

# Constants: Link lengths and platform radius
l1, l2, l3 = 80.0, 50.0, 64.5  # mm
R = 100.0  # mm
PWM_RANGES = [(950, 2200), (1300, 2450), (990, 2200)]
theta_ranges = [-90, 74]
NEUTRAL_PWM = [1575, 1860, 1615]

def set_pwm(channel, pwm_value):
    """Send a PWM value to a specific servo channel."""
    target_value = int(pwm_value * 4)
    command = [0x84, channel, target_value & 0x7F, (target_value >> 7) & 0x7F]
    ser.write(bytearray(command))
    rospy.loginfo(f"Set PWM {pwm_value} on channel {channel}")

def calculate_theta_for_motor(alpha, beta, z_height, motor_id):
    """Calculate the motor angle θ for pitch, roll, and Z height."""
    angle_offset = math.radians(motor_id * 120)
    alpha_rad, beta_rad = math.radians(alpha), math.radians(beta)

    h_adjusted = z_height + R * (
        math.sin(beta_rad) * math.cos(angle_offset) +
        math.sin(alpha_rad) * math.sin(angle_offset)
    )

    A, B = 2 * h_adjusted * l2, 2 * R * l2 - 2 * l2 * l3
    C = h_adjusted**2 - l1**2 + l2**2 + l3**2 + R**2 - 2 * R * l3
    D = -math.sqrt(A**2 + B**2)

    phi = math.atan2(B, A)
    theta = math.asin(C / D) + phi

    rospy.loginfo(f"Motor {motor_id} - Theta: {math.degrees(theta):.2f}, Height: {h_adjusted:.2f}")
    return -math.degrees(theta)

def map_range(value, min_in, max_in, min_out, max_out):
    """Map a value from one range to another."""
    return (value - min_in) / (max_in - min_in) * (max_out - min_out) + min_out

def theta_to_pwm(thetas):
    """Convert motor angles θ to corresponding PWM values."""
    pwm_values = []
    for i, theta in enumerate(thetas):
        angle = -theta if i == 1 else theta
        min_pwm, max_pwm = PWM_RANGES[i]
        pwm = map_range(angle, theta_ranges[0], theta_ranges[1], min_pwm, max_pwm)
        pwm_values.append(max(min_pwm, min(max_pwm, pwm)))
    return pwm_values

def update_motors(data):
    """Callback to update motors based on the desired orientation."""
    alpha, beta, z_height = data.data
    thetas = [calculate_theta_for_motor(alpha, beta, z_height, i) for i in range(3)]
    pwm_values = theta_to_pwm(thetas)

    for i, pwm in enumerate(pwm_values):
        threading.Thread(target=set_pwm, args=(i, pwm)).start()

def motor_control_node():
    """ROS node for motor control."""
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.Subscriber('/desired_orientation', Float32MultiArray, update_motors)
    rospy.loginfo("Motor control node started, waiting for commands...")
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        ser.close()
        rospy.loginfo("Motor control node stopped.")
