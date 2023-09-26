"""
Env 2D
@author: huiming zhou
"""
import os
import sys
# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))

parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

from obstacle import radius_list, middle_point_list, x_range, y_range, motions

def create_circle(middle_point, radius, x_range, y_range) -> list:
    # Calculate the square of the radius to avoid using sqrt for comparison
    radius_squared = radius ** 2

    # Initialize the result list
    elements_inside_circle = []

    lower_bound_x = max(middle_point[0] - radius, 0)
    upper_bound_x = min(middle_point[0] + radius, x_range)
    lower_bound_y = max(middle_point[1] - radius, 0)
    upper_bound_y = min(middle_point[1] + radius, y_range)

    for x in range(lower_bound_x, upper_bound_x):
        for y in range(lower_bound_y, upper_bound_y):
            distance_squared = (x - middle_point[0]) ** 2 + (y - middle_point[1]) ** 2

            # Check if the current point (x, y) is inside the circle
            if distance_squared <= radius_squared:
                elements_inside_circle.append((x, y))
    
    return elements_inside_circle

class Env:
    def __init__(self):
        self.x_range = x_range  # size of background
        self.y_range = y_range
        self.motions = motions
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(0, len(radius_list)):
            obs.update(create_circle(middle_point_list[i], radius_list[i], x, y))

        return obs