import tkinter as tk
import serial
import threading
import math

# Initialize serial connection to Pololu Maestro
try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    print("Connected to Pololu Maestro on /dev/serial0")
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

# Constants: Link lengths and platform radius
l1 = 80.0  # mm
l2 = 50.0  # mm
l3 = 64.5  # mm
R = 100.0  # mm

# PWM ranges and neutral points
NEUTRAL_PWM = [1575, 1860, 1615]
PWM_RANGES = [(950, 2200), (1300, 2450), (990, 2200)]
theta_ranges = [-90, 74]  # Theta min and max

def set_pwm(channel, pwm_value):
    """Send a PWM value to a specific servo channel."""
    target_value = int(pwm_value * 4)
    command = [0x84, channel, target_value & 0x7F, (target_value >> 7) & 0x7F]
    ser.write(bytearray(command))
    print(f"Set PWM {pwm_value} on channel {channel}")

def calculate_theta_for_motor(alpha, beta, z_height, motor_id):
    """Calculate the motor angle θ for a given motor with pitch (α), roll (β), and Z height."""
    # Calculate the motor’s angular offset (0° for motor 1, ±120° for motors 2 and 3)
    angle_offset = math.radians(motor_id * 120)

    # Convert pitch and roll to radians
    alpha_rad = math.radians(alpha)  # Pitch (rotation about Y-axis)
    beta_rad = math.radians(beta)    # Roll (rotation about X-axis)

    # Adjust the Z height based on pitch and roll for each motor
    h_adjusted = z_height + R * (
        math.sin(beta_rad) * math.cos(angle_offset) +
        math.sin(alpha_rad) * math.sin(angle_offset)
    )

    # Compute A, B, C, D, and φ using the adjusted height
    A = 2 * h_adjusted * l2
    B = 2 * R * l2 - 2 * l2 * l3
    C = h_adjusted**2 - l1**2 + l2**2 + l3**2 + R**2 - 2 * R * l3
    D = -math.sqrt(A**2 + B**2)  # Negative to follow your working example

    # Calculate the motor angle θ using the provided formula
    phi = math.atan2(B, A)
    theta = math.asin(C / D) + phi

    print(f"Motor {motor_id} - Theta: {math.degrees(theta):.2f} degrees, Adjusted Height: {h_adjusted:.2f}")
    return -math.degrees(theta)


def map_range(value, min_in, max_in, min_out, max_out):
    """Map a value from one range to another."""
    return (value - min_in) / (max_in - min_in) * (max_out - min_out) + min_out

def theta_to_pwm(thetas):
    """Convert a list of motor angles θ to corresponding PWM values."""
    pwm_values = []
    for i, theta in enumerate(thetas):
        # Handle inverted servo (Servo 2)
        angle = -theta if i == 1 else theta

        # Map θ to the corresponding PWM value
        min_pwm, max_pwm = PWM_RANGES[i]
        pwm = map_range(angle, theta_ranges[0], theta_ranges[1], min_pwm, max_pwm)

        # Clamp the PWM within the valid range
        pwm = max(min_pwm, min(max_pwm, pwm))
        pwm_values.append(pwm)

    return pwm_values

def update_motors(alpha, beta, z_height):
    """Update motor positions based on pitch, roll, and Z height."""
    thetas = [calculate_theta_for_motor(alpha, beta, z_height, i) for i in range(3)]
    pwm_values = theta_to_pwm(thetas)

    # Send PWM signals to all three servos
    for i, pwm in enumerate(pwm_values):
        threading.Thread(target=set_pwm, args=(i, pwm)).start()



def on_slider_change(*args):
    """Callback for slider changes."""
    alpha = alpha_slider.get()
    beta = beta_slider.get()
    z_height = z_slider.get()
    update_motors(alpha, beta, z_height)

def set_home():
    """Set all servos to their home positions."""
    for i, pwm in enumerate(NEUTRAL_PWM):
        set_pwm(i, pwm)

def on_exit():
    """Close the serial connection on exit."""
    ser.close()
    print("Serial connection closed.")

# GUI setup
root = tk.Tk()
root.title("3-RRS Platform Control")
root.geometry("800x600")

# α (pitch) slider
tk.Label(root, text="Pitch (α)", font=("Arial", 16)).pack(pady=10)
alpha_slider = tk.Scale(root, from_=-45, to=45, orient='horizontal', length=600,
                        font=("Arial", 14), command=on_slider_change)
alpha_slider.set(0)  # Default to 0 degrees
alpha_slider.pack(pady=10)

# β (roll) slider
tk.Label(root, text="Roll (β)", font=("Arial", 16)).pack(pady=10)
beta_slider = tk.Scale(root, from_=-45, to=45, orient='horizontal', length=600,
                       font=("Arial", 14), command=on_slider_change)
beta_slider.set(0)  # Default to 0 degrees
beta_slider.pack(pady=10)

# Z height slider
tk.Label(root, text="Z Height", font=("Arial", 16)).pack(pady=10)
z_slider = tk.Scale(root, from_=21.69, to=125.06, orient='horizontal', length=600,
                    font=("Arial", 14), command=on_slider_change)
z_slider.set((21.69 + 125.06) / 2)  # Set to middle height by default
z_slider.pack(pady=10)

# Home button
home_button = tk.Button(root, text="Home", font=("Arial", 14), command=set_home)
home_button.pack(pady=20)

# Handle window close
root.protocol("WM_DELETE_WINDOW", on_exit)

# Run GUI loop
root.mainloop()
