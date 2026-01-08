
import numpy as np

def generate_figure_eight(width, height, num_points=200, filename="trajectory.txt"):
    """
    Generates an 8-shaped trajectory (Lemniscate) fitted within a bounding box.
    """
    # Create a time array from 0 to 2*PI
    t = np.linspace(0, 2 * np.pi, num_points)

    # Parametric equations for a figure-eight (Lemniscate of Gerono)
    # We use these because they map cleanly to a rectangular bounding box
    x = np.sin(t)
    y = np.sin(t) * np.cos(t)

    # Normalize coordinates to range [0, 1]
    x_norm = (x - x.min()) / (x.max() - x.min())
    y_norm = (y - y.min()) / (y.max() - y.min())

    # Scale to the desired space (width x height)
    x_scaled = x_norm * width
    y_scaled = y_norm * height

    # Save to file in "x,y" format
    with open(filename, "w") as f:
        for xi, yi in zip(x_scaled, y_scaled):
            # Formatting to 2 decimal places for cleanliness
            f.write(f"{xi:.2f},{yi:.2f}\n")

    print(f"Successfully generated {num_points} points to {filename}")

# Configuration
SPACE_WIDTH = 60
SPACE_HEIGHT = 60
POINTS = 50  # Number of steps in the simulation

generate_figure_eight(SPACE_WIDTH, SPACE_HEIGHT, POINTS)
