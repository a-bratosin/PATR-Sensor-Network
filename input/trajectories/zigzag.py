import numpy as np
import argparse

def generate_zigzag(width, height, num_points=200, filename="trajectory.txt", periods=5):
    """
    Generates a zigzag trajectory within a bounding box.
    """
    # Create a linear progression from 0 to 1 for the horizontal axis (X)
    t = np.linspace(0, 1, num_points)
    
    # X progresses linearly across the width
    x_scaled = t * width
    
    # Y uses a triangle wave to create the zigzag effect across the height
    # Formula for triangle wave: (2/pi) * arcsin(sin(2 * pi * frequency * t))
    # We use a simplified version using the absolute value of a sawtooth
    raw_zigzag = 2 * np.abs(periods * t - np.floor(periods * t + 0.5))
    
    # Scale Y to the desired height
    y_scaled = raw_zigzag * height

    # Save to file in "x,y" format
    try:
        with open(filename, "w") as f:
            for xi, yi in zip(x_scaled, y_scaled):
                f.write(f"{xi:.2f},{yi:.2f}\n")
        print(f"Successfully generated {num_points} points with {periods} zigzags to {filename}")
    except IOError as e:
        print(f"Error writing to file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a zigzag trajectory file for PATR.")
    
    # Positional arguments
    parser.add_argument("width", type=float, help="The total horizontal distance covered")
    parser.add_argument("height", type=float, help="The vertical amplitude of the zigzag")
    
    # Optional arguments
    parser.add_argument("--points", type=int, default=100, help="Number of points (default: 100)")
    parser.add_argument("--output", type=str, default="trajectory.txt", help="Output filename")
    parser.add_argument("--periods", type=int, default=5, help="Number of 'zags' (default: 5)")

    args = parser.parse_args()

    # Execute the function
    generate_zigzag(args.width, args.height, args.points, args.output, args.periods)
