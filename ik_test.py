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

# Constants: Updated link lengths
l1 = 80.0  # mm
l2 = 50.0  # mm
l3 = 64.5  # mm
R = 100.0  # mm (updated platform radius)

# PWM ranges and neutral points
NEUTRAL_PWM = [1575, 1860, 1615]
PWM_RANGES = [(950, 2200), (1300,2450), (990, 2200)]
theta_ranges = [-90,74]
def set_pwm(channel, pwm_value):
    """Send a PWM value to a specific servo channel."""
    target_value = int(pwm_value * 4)
    command = [0x84, channel, target_value & 0x7F, (target_value >> 7) & 0x7F]
    ser.write(bytearray(command))
    print(f"Set PWM {pwm_value} on channel {channel}")

def calculate_theta_for_z(z_height):
    """Calculate the motor angle θ for a given Z height."""
    h = z_height

    # Compute A, B, C, D, and φ
    A = 2 * h * l2
    B = 2 * R * l2 - 2 * l2 * l3
    C = h**2 - l1**2 + l2**2 + l3**2 + R**2 - 2 * R * l3
    D = -math.sqrt(A**2 + B**2)

    # Calculate θ using the provided formula
    phi = math.atan2(B, A)
    theta = math.asin(C / D) + phi

    print(f"Calculated Theta: θ = {(math.degrees(theta)):.2f} degrees, h: {h}")
    return -math.degrees(theta)

def map_range(value, min_in, max_in, min_out, max_out):
    """Map a value from one range to another."""
    return (value - min_in) / (max_in - min_in) * (max_out - min_out) + min_out

def theta_to_pwm(theta):
    """Convert motor angle θ to corresponding PWM values."""
    pwm_values = []
    for i, (min_pwm, max_pwm) in enumerate(PWM_RANGES):
        # Handle inverted servo (Servo 2)
        angle = -theta if i == 1 else theta  # Invert θ for the second servo

        # Map θ to the corresponding PWM value
        pwm = map_range(angle, theta_ranges[0], theta_ranges[1], min_pwm, max_pwm)

        # Clamp the PWM within the valid range
        #pwm = max(min_pwm, min(max_pwm, pwm))
        pwm_values.append(pwm)

    return pwm_values


def update_motors(z_height):
    """Update motor positions based on Z height."""
    theta = calculate_theta_for_z(z_height)
    pwm_values = theta_to_pwm(theta)

    # Send PWM signals to all three servos
    for i, pwm in enumerate(pwm_values):
        threading.Thread(target=set_pwm, args=(i, pwm)).start()

def on_slider_change(*args):
    """Callback for slider changes."""
    z_height = z_slider.get()
    update_motors(z_height)

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
root.title("3-RRS Platform Z-Height Control")
root.geometry("800x400")

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
