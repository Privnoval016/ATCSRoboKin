import matplotlib.pyplot as plt
import numpy as np

# Circle parameters
radius = 1
center = (0, 0)
total_points = 100  # Total number of points to place on the circle
duration = 10  # Total duration in seconds
interval = duration / total_points  # Time interval for each point

# Create a figure and axis
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')

# Draw the circle's outline
circle_outline = plt.Circle(center, radius, color='lightgray', fill=False)
ax.add_artist(circle_outline)

# Initialize the points list
x_points = []
y_points = []

# Show the plot
plt.show(block=False)

# Update loop to add points incrementally
for i in range(total_points):
    angle = 2 * np.pi * i / total_points  # Calculate angle for the point
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle)
    x_points.append(x)
    y_points.append(y)

    # Draw the updated points
    ax.plot(x_points, y_points, 'ro')  # Plot all points up to the current step
    plt.pause(interval)  # Pause to control the timing

plt.ioff()  # Turn off interactive mode
plt.show()
