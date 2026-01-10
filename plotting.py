import matplotlib.pyplot as plt
import numpy as np

# File paths
sensor_file = "input/sensor_positions/random_30p_60x60.txt"
beacon_file = "input/trajectories/ellipse_60x60_50p.txt"
estimation_file = "estimated1.txt"

# Function to read points from file
def read_points(filename):
    data = np.loadtxt(filename, delimiter=',')
    x, y = data[:,0], data[:,1]
    return x, y

# Read points
sensor_x, sensor_y = read_points(sensor_file)
beacon_x, beacon_y = read_points(beacon_file)
estimation_x, estimation_y = read_points(estimation_file)

# Create figure
plt.figure(figsize=(8,8))
plt.xlim(0,60)
plt.ylim(0,60)
plt.grid(True, linestyle='--', alpha=0.5)

# Plot sensors as dotted circles
plt.scatter(sensor_x, sensor_y, s=100, facecolors='none', edgecolors='blue', label='Sensors')
for x, y in zip(sensor_x, sensor_y):
    circle = plt.Circle((x, y), 1.0, color='blue', fill=False, linestyle='dotted', alpha=0.7)
    plt.gca().add_patch(circle)

# Plot beacon points with connected lines
plt.plot(beacon_x, beacon_y, '-o', color='red', label='Beacon', markersize=6, linewidth=2)

# Plot estimation points with connected lines
plt.plot(estimation_x, estimation_y, '-s', color='green', label='Estimation', markersize=6, linewidth=2)

# Labels and legend
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.title("Sensor Network Localization")
plt.legend()
plt.axis('equal')

# Save the figure
plt.savefig("random_ellipse_50_plot.png", dpi=300, bbox_inches='tight')

# Show plot
plt.show()
