import numpy as np
import argparse

def generate_triangular_grid(width, height, spacing, filename="anchors.txt"):
    """
    Generates a grid of anchor nodes where each set of 3 neighbors 
    forms an equilateral triangle.
    """
    anchors = []
    
    # In an equilateral triangle with side 's':
    # The horizontal distance between nodes in a row is 's'
    # The vertical distance between rows is s * sqrt(3) / 2
    row_spacing = spacing * (np.sqrt(3) / 2)
    
    # Calculate number of rows and columns
    num_rows = int(height / row_spacing) + 1
    num_cols = int(width / spacing) + 1
    
    for row in range(num_rows):
        y = row * row_spacing
        # Offset every second row by half the spacing to create the triangle pattern
        offset = (spacing / 2) if (row % 2 == 1) else 0
        
        for col in range(num_cols):
            x = col * spacing + offset
            
            # Keep nodes within the bounding box
            if x <= width:
                anchors.append((x, y))

    # Save to file
    try:
        with open(filename, "w") as f:
            for xi, yi in anchors:
                f.write(f"{xi:.2f},{yi:.2f}\n")
        print(f"Generated {len(anchors)} nodes in a triangular grid (spacing: {spacing}) to {filename}")
    except IOError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a triangular grid of anchor nodes.")
    
    parser.add_argument("width", type=float, help="Width of the field")
    parser.add_argument("height", type=float, help="Height of the field")
    parser.add_argument("spacing", type=float, help="Distance between adjacent nodes")
    parser.add_argument("--output", type=str, default="anchors_grid.txt", help="Output filename")

    args = parser.parse_args()
    generate_triangular_grid(args.width, args.height, args.spacing, args.output)
