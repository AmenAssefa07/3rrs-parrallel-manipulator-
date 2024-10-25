import math

# Constants: Link lengths and platform radius
l1 = 80.0  # mm
l2 = 50.0  # mm
l3 = 64.5  # mm
R = 100.0  # mm (updated platform radius)

def calculate_theta_for_z(z_height):
    """Calculate two motor angles θ1 and θ2 for a given Z height."""
    h = z_height

    # Compute A, B, C
    A = 2 * h * l2
    B = 2 * R * l2 - 2 * l2 * l3
    C = h**2 - l1**2 + l2**2 + l3**2 + R**2 - 2 * R * l3

    # Calculate D with positive and negative roots
    D_pos = math.sqrt(A**2 + B**2)
    D_neg = -math.sqrt(A**2 + B**2)

    # Calculate θ1 and θ2 using the provided formula
    phi = math.atan2(B, A)

    theta1 = math.degrees(math.asin(C / D_pos) + phi)
    theta2 = math.degrees(math.asin(C / D_neg) + phi)

    return theta1, theta2

def main():
    print("Enter Z height values (mm). Type 'exit' to quit.")
    while True:
        try:
            # Get height input from user
            user_input = input("Enter Z height: ")

            # Exit the loop if the user types 'exit'
            if user_input.lower() == 'exit':
                break

            # Convert input to float and calculate the two theta values
            z_height = float(user_input)
            theta1, theta2 = calculate_theta_for_z(z_height)

            # Print the results
            print(f"Height: {z_height} mm, Theta1: {theta1:.2f} degrees, Theta2: {theta2:.2f} degrees")

        except ValueError:
            print("Invalid input. Please enter a valid number or 'exit' to quit.")

if __name__ == "__main__":
    main()
