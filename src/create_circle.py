#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy

# Parameters
radius = 100
num_points = 100  # Number of points on the circle
center = (100, 200)  # Center of the circle

# Calculate evenly spaced angles
angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

# Calculate coordinates of points on the circle
x_coords = center[0] + radius * np.cos(angles)
y_coords = [200 for _ in range(num_points)]
z_coords = center[1] + radius * np.sin(angles)

# Combine x and y coordinates to get points
circle_points = np.column_stack((x_coords, y_coords, z_coords))
circle_points = [[float(i) for i in point] for point in circle_points]

# Print the coordinates of the points
print("Circle Points: ")
print(circle_points)

# Publish the points
rospy.init_node('circle_publisher')
rospy.set_param('/arm_movement', circle_points)




# Optional: Visualize the circle
# plt.figure(figsize=(6, 6))
# plt.scatter(x_coords, z_coords, color='blue')
# plt.plot(x_coords, z_coords, linestyle='--', color='gray')
# plt.gca().set_aspect('equal', adjustable='box')
# plt.title('Points on a Circle')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.grid()
# plt.show()
