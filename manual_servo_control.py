"""
PWM Servo Control GUI for 3 Servos using Pololu Maestro and Raspberry Pi

This script provides a graphical interface to control three servos connected 
via the Pololu Maestro. This script is mostly meant to help define the pwm ranges
for each servo.

Features:
- Non-blocking serial communication using threads.
- Customizable servo ranges (700 - 3000 PWM).
- Graceful shutdown to close the serial connection.

Dependencies:
- Python 3
- pyserial
"""


import tkinter as tk
import serial
import threading

# Initialize the serial connection to Pololu Maestro
try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    print("Connected to Pololu Maestro on /dev/serial0")
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

# Custom home PWM values for each servo
HOME_PWM = [1575, 1860, 1615]

def set_pwm(channel, pwm_value):
    """Send a PWM value to a specific servo channel."""
    target_value = int(pwm_value * 4)   # Convert PWM to quarter-microseconds for the Pololu Maestro
    command = [0x84, channel, target_value & 0x7F, (target_value >> 7) & 0x7F]
    ser.write(bytearray(command))
    print(f"Set PWM {pwm_value} on channel {channel}")

def update_servo(channel, slider_value):
    """Callback to update the servo position based on slider input."""
    pwm_value = int(slider_value)
    threading.Thread(target=set_pwm, args=(channel, pwm_value)).start()

def increment_pwm(step):
    """Increment or decrement PWM for all servos by the given step."""
    for i, slider in enumerate(sliders):
        current_value = slider.get()
        # Inverse control for the second servo (channel 1)
        if i == 1:
            new_value = max(700, min(3000, current_value - step))
        else:
            new_value = max(700, min(3000, current_value + step))
        
        slider.set(new_value)  # Update the slider position
        update_servo(i, new_value)  # Update the servo

def set_home():
    """Set all servos to their respective home positions."""
    for i, slider in enumerate(sliders):
        slider.set(HOME_PWM[i])  # Set slider to the custom home value
        update_servo(i, HOME_PWM[i])  # Update the servo

def on_exit():
    """Close the serial connection when exiting."""
    ser.close()
    print("Serial connection closed.")

# GUI setup
root = tk.Tk()
root.title("PWM Servo Control")
root.geometry("800x600")  # Adjusted window size

sliders = []  # Store references to sliders for easy access

# Create sliders with larger size and labels
for i in range(3):
    tk.Label(root, text=f"Servo {i+1} PWM Value", font=("Arial", 16)).pack(pady=10)
    slider = tk.Scale(
        root, from_=700, to=3000, orient='horizontal',
        length=600, font=("Arial", 14),
        command=lambda value, ch=i: update_servo(ch, value)
    )
    slider.set(HOME_PWM[i])  # Set default to the custom home PWM
    slider.pack(pady=15)
    sliders.append(slider)

# Create increment and decrement buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

increment = 25

increment_button = tk.Button(button_frame, text="+ PWM", font=("Arial", 14),
                             command=lambda: increment_pwm(increment))
increment_button.grid(row=0, column=0, padx=10)

decrement_button = tk.Button(button_frame, text="- PWM", font=("Arial", 14),
                             command=lambda: increment_pwm(-increment))
decrement_button.grid(row=0, column=1, padx=10)

home_button = tk.Button(root, text="Home", font=("Arial", 14), command=set_home)
home_button.pack(pady=10)

# Run the GUI event loop and handle exit
root.protocol("WM_DELETE_WINDOW", on_exit)
root.mainloop()
