import numpy as np
import argparse

def generate_perimeter_anchors(width, height, k, filename="anchors.txt"):
    """
    Distributes k anchor nodes evenly along the perimeter of a rectangle.
    """
    perimeter = 2 * (width + height)
    spacing = perimeter / k
    anchors = []

    for i in range(k):
        # Current distance along the perimeter
        d = i * spacing
        
        if d <= width:
            # Bottom edge (moving right)
            x, y = d, 0
        elif d <= width + height:
            # Right edge (moving up)
            x, y = width, d - width
        elif d <= 2 * width + height:
            # Top edge (moving left)
            x, y = width - (d - (width + height)), height
        else:
            # Left edge (moving down)
            x, y = 0, height - (d - (2 * width + height))
            
        anchors.append((x, y))

    # Save to file in "x,y" format
    try:
        with open(filename, "w") as f:
            for xi, yi in anchors:
                f.write(f"{xi:.2f},{yi:.2f}\n")
        print(f"Successfully generated {k} anchor nodes along the perimeter to {filename}")
    except IOError as e:
        print(f"Error writing to file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate perimeter anchor node coordinates.")
    
    # Positional arguments
    parser.add_argument("width", type=float, help="Width of the sensor field")
    parser.add_argument("height", type=float, help="Height of the sensor field")
    parser.add_argument("k", type=int, help="Number of anchor nodes (Nk)")
    
    # Optional arguments
    parser.add_argument("--output", type=str, default="anchors.txt", help="Output filename")

    args = parser.parse_args()

    generate_perimeter_anchors(args.width, args.height, args.k, args.output)
