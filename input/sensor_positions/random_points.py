import numpy as np
import argparse

def generate_random_anchors(width, height, k, filename="anchors_random.txt"):
    """
    Generates k anchor nodes at random positions within the bounding box.
    """
    # Generate random X and Y coordinates using a uniform distribution
    x_coords = np.random.uniform(0, width, k)
    y_coords = np.random.uniform(0, height, k)
    
    anchors = list(zip(x_coords, y_coords))

    # Save to file in "x,y" format
    try:
        with open(filename, "w") as f:
            for xi, yi in anchors:
                f.write(f"{xi:.2f},{yi:.2f}\n")
        print(f"Successfully generated {k} random anchor nodes to {filename}")
    except IOError as e:
        print(f"Error writing to file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate random anchor node coordinates for PATR baseline.")
    
    # Positional arguments
    parser.add_argument("width", type=float, help="Width of the sensor field")
    parser.add_argument("height", type=float, help="Height of the sensor field")
    parser.add_argument("k", type=int, help="Number of anchor nodes (Nk) to generate")
    
    # Optional arguments
    parser.add_argument("--output", type=str, default="anchors_random.txt", help="Output filename")

    args = parser.parse_args()

    generate_random_anchors(args.width, args.height, args.k, args.output)
