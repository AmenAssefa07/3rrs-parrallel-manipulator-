import math

# Constants: Link lengths in millimeters
l1 = 80.0  # Distance from center to motor joint (mm)
l2 = 50.0  # Length between two revolute joints (mm)
l3 = 64.5  # Length from revolute to spherical joint (mm)
r = 100.0  # Radius of the platform (mm)

def calculate_h(theta):
    """Calculate the h values based on the given theta."""
    # Convert theta to radians
    theta_rad = math.radians(theta)

    # Calculate quadratic components
    A = 1
    B = 2 * l2 * math.sin(theta_rad)
    C = -(l1**2 - l2**2 + 2*r*(l3+l2*math.cos(theta_rad)) - r**2 - 2*l3*(l3+l2*math.cos(theta_rad)) +l3**2)

    # Calculate the discriminant
    discriminant = B**2 - 4*C

    # Check if the discriminant is negative
    if discriminant < 0:
        return float('nan'), float('nan'), discriminant  # No real solutions

    # Calculate the two solutions for h
    h1 = (B + math.sqrt(discriminant)) / 2
    h2 = (B - math.sqrt(discriminant)) / 2

    return h1, h2, discriminant

# Loop through angles from -180 to 180 degrees and print results
for theta in range(-180, 181):
    h1, h2, d = calculate_h(theta)
    print(f"Theta: {theta}°, h1: {h1:.2f}, h2: {h2:.2f}, d: {d:.2f}")
