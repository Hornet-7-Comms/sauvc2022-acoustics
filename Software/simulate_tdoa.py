from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt
import numpy as np
#import math as m
import sys

from hornet_acoustics_library import *

# Parameters
hx = 40
hy = 42
hz = 30
depth = 100
acoustics = Acoustics(hx, hy, hz)

fig = plt.figure(figsize=(16,9))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("x axis")
ax.set_ylabel("y axis")
ax.set_zlabel("z axis")
ax.set_proj_type('ortho')


def plot_dot(x, y, z, **kwargs):
    ax.scatter(x, y, zs=z, s=100, **kwargs)

# Plot hydrophones dot
hydrophones_colors = ['r', 'b', 'g', 'm']
hydrophones_text = ['0', '1', '2', '3']

for i in range(4):
    plot_dot(*acoustics.hydrophone_positions[i], color=hydrophones_colors[i])
    ax.text(*acoustics.hydrophone_positions[i], hydrophones_text[i], size=18, zorder=1, color='black')

# Plot vehicle wireframe
vehicle_vertices = np.array([
    [[0, 0, 0], [hx, 0, 0], [hx, hy, 0], [0, hy, 0]], # Bottom
    [[0, 0, hz], [hx, 0, hz], [hx, hy, hz], [0, hy, hz]], # Top
    [[0, 0, hz], [hx, 0, hz], [hx, 0, 0], [0, 0, 0]], # Front
    [[0, hy, hz], [hx, hy, hz], [hx, hy, 0], [0, hy, 0]], # Back
    [[hx, hy, 0], [hx, hy, hz], [hx, 0, hz], [hx, 0, 0]], # Left
    [[0, hy, 0], [0, hy, hz], [0, 0, hz], [0, 0, 0]], # Right
])
ax.add_collection3d(Poly3DCollection(vehicle_vertices, 
 facecolors='yellow', linewidths=0.5, edgecolors='black', alpha=.15))

shadow_vertices = np.array([
    [[0, 0, -depth], [hx, 0, -depth], [hx, hy, -depth], [0, hy, -depth]], # Bottom
])
ax.add_collection3d(Poly3DCollection(shadow_vertices, 
 facecolors='grey', linewidths=0.5, edgecolors='black', alpha=.15))

#plt.show()

# Plot target dot
target = np.array((-398, -374, -depth))
target = np.array((5098, 1874, -depth))
target = np.array((300, 100, -depth))
angle_target = int(sys.argv[1])
distance_target = 400
target = np.array([distance_target*np.sin(np.radians(angle_target+180)),
                   distance_target*np.cos(np.radians(angle_target+180)),
                   -depth])

plt.xlim([-1.1*distance_target, 1.1*distance_target])
plt.ylim([-1.1*distance_target, 1.1*distance_target])
plot_dot(*target, color='black')
print("Target coordinates", target)

# Plot lines to target
wave_vertices = np.array([
    [acoustics.hydrophone_positions[0], target],
    [acoustics.hydrophone_positions[1], target],
    [acoustics.hydrophone_positions[2], target],
    [acoustics.hydrophone_positions[3], target],
])
ax.add_collection3d(Poly3DCollection(wave_vertices, 
 facecolors='white', linewidths=1, edgecolors=hydrophones_colors, alpha=.15))

# Calculations
distances = list(map(lambda ab: np.linalg.norm(np.array(ab[0]) - np.array(ab[1])), wave_vertices))
print("Distances: ", distances)
print("Diff: ", distances - min(distances))

# Angle of elevation
"""
Use stacked hydrophone pair to extimate angle of elevation.
Angle is more accurate as the pinger get closer to the vehicle
--> angle_elevation = arcsin(tdoa_vertical / distance_vertical)
"""
delta_target = np.array(acoustics.hydrophone_positions[0]) - np.array(target)
angle_elevation = np.arcsin(delta_target[2] / np.linalg.norm(delta_target))
angle_elevation = np.degrees(angle_elevation)
print("Angle of elevation (actual):", angle_elevation)

acoustics.tdoa_distance_list = distances
angle_elevation = acoustics.get_elevation_angle()
print("Angle of elevation (tdoa):", angle_elevation)

# Plot extimated radius of pinger location
"""
Likewise, if we have depth sensor, we can estimate radius of pinger
Distance estimate is more accurate as the pinger get closer to the vehicle
--> depth / sin(angle_elevation)
"""
def plot_circle(center_x, center_y, center_z, radius, **kwargs):
    _r = radius
    theta = np.linspace(0., 4*np.pi/2., 201)
    _x = _r*np.sin(theta) + center_x
    _y = _r*np.cos(theta) + center_y
    _z = np.ones_like(theta) * center_z
    ax.plot(_x, _y, _z, **kwargs)

target_extimated_radius = acoustics.estimate_distance_to_pinger(depth)
print("target_extimated_radius", target_extimated_radius)
plot_circle(0, 0, -depth, target_extimated_radius, linestyle='dashed')

# Angle of bearing
"""
For bearing angle, we take the first and last triggered hydrophone
since it produces the greatest length and thus it reduces the margin of error.

The calculated bearing angle uses the TDOA between the hydrophone pair, 
with normalisation done using the elevation angle to get the projected length
(TDOA provides adjacent, while hypotheneus should be used which is parallel to the poolbed).

"""
#forward_vector = hydrophones_positions[0] - hydrophones_positions[2]
#forward_vector_length = np.linalg.norm(hydrophones_positions[0] - hydrophones_positions[2])
#trigger_first_index = trigger_sorted_index[0]#np.argmin(distances[:-1]) # exclude stacked hydrophone
#trigger_last_index = trigger_sorted_index[1]#np.argmax(distances[:-1]) # exclude stacked hydrophone
#offset_angle = np.degrees(np.arccos(pair_hydrophone_vector.dot(forward_vector) / forward_vector_length / pair_hydrophone_length))

print(acoustics.get_bearing_angle())
angle_bearing = acoustics.get_bearing_angle()['angle']

ax.quiver(0, 0, 0, -100*np.sin(np.radians(angle_bearing)), -100*np.cos(np.radians(angle_bearing)), 0)
ax.quiver(0, 0, -depth, -100*np.sin(np.radians(angle_bearing)), -100*np.cos(np.radians(angle_bearing)), 0)

ax.text(hx/2, hy/2, 0, acoustics.get_bearing_angle()['quadrant'], size=16, zorder=1, color='black', ha='center', va='center')


# ax_limits = []
# ax_limits.extend(ax.get_xlim())
# ax_limits.extend(ax.get_ylim())
# ax_limits.extend(ax.get_zlim())
# ax.set_xlim([min(ax_limits), max(ax_limits)])
# ax.set_ylim([min(ax_limits), max(ax_limits)])
# ax.set_zlim([min(ax_limits)*0.33, max(ax_limits)*0.33])

ax.view_init(azim=0, elev=90)
ax.set_box_aspect(aspect=(3,3,1))
plt.tight_layout()
plt.subplots_adjust(top=1.3,
                    bottom=-0.3,
                    left=0.0,
                    right=1.0,
                    hspace=0.2,
                    wspace=0.2)

try:
    if sys.argv[2]:
        plt.savefig(f'my_plot_{angle_target:03d}.png')
    else:
        plt.show()
except:
    plt.show()
