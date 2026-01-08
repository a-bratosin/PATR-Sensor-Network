import numpy as np
import argparse
import sys

def generate_ellipse(width, height, num_points=200, filename="trajectory.txt"):
    """
    Generates an elliptical trajectory fitted within a bounding box.
    """
    # Create a time array from 0 to 2*PI
    t = np.linspace(0, 2 * np.pi, num_points)

    # Parametric equations for an ellipse
    # Cosine for X and Sine for Y creates a smooth loop
    x = np.cos(t)
    y = np.sin(t)

    # Normalize coordinates to range [0, 1]
    # This ensures the shape perfectly fits the width/height provided
    x_norm = (x - x.min()) / (x.max() - x.min())
    y_norm = (y - y.min()) / (y.max() - y.min())

    # Scale to the desired space (width x height)
    x_scaled = x_norm * width
    y_scaled = y_norm * height

    # Save to file in "x,y" format
    try:
        with open(filename, "w") as f:
            for xi, yi in zip(x_scaled, y_scaled):
                f.write(f"{xi:.2f},{yi:.2f}\n")
        print(f"Successfully generated {num_points} points to {filename}")
    except IOError as e:
        print(f"Error writing to file: {e}")

if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Generate an elliptical trajectory file.")
    
    parser.add_argument("width", type=float, help="The width of the bounding box")
    parser.add_argument("height", type=float, help="The height of the bounding box")
    parser.add_argument("--points", type=int, default=50, help="Number of points (default: 50)")
    parser.add_argument("--output", type=str, default="trajectory.txt", help="Output filename (default: trajectory.txt)")

    args = parser.parse_args()

    # Execute the function with command line arguments
    generate_ellipse(args.width, args.height, args.points, args.output)
