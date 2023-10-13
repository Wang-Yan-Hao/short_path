import math
import os
import sys

# Multiplier of distance for UAV run out of obstacles
run_distance_multiplier = 1.2

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, "..")
sys.path.append(parent_dir)
parent_dir = os.path.join(current_dir, ".")
sys.path.append(parent_dir)

from utility import calculate_distance, point_in_obstacles, vector_normal

from Search_2D.tangent_method.cut_line import main


# Determine p1 and p2 is in same obstacle circumference or not
def same_obstacle(obs, rds, p1, p2):
    for index, element in enumerate(obs):
        if (
            calculate_distance(p1, element) <= rds[index]
            and calculate_distance(p2, element) <= rds[index]
        ):
            return index  # return the index of the obstacle
    return -1


# Convert point coordinates to polar coordinates
def cartesian_to_polar(x, y, center):
    x0, y0 = center
    theta = math.atan2(y - y0, x - x0)
    return theta


# Find the point with length to point_2, it should between point_1 and point_2
def find_next_point_on_circle(center, rds, point_1, point_2, length):
    # Convert Cartesian coordinates to polar coordinates
    theta1 = cartesian_to_polar(point_1[0], point_1[1], center)
    theta2 = cartesian_to_polar(point_2[0], point_2[1], center)
    delta_theta = theta2 - theta1

    # Circumference
    circumference = rds * 2 * math.pi

    theta_result = theta1
    if delta_theta < 0:
        # Calculate the angle of the point on the arc
        theta_result -= length / circumference * 2 * math.pi
    else:
        theta_result += length / circumference * 2 * math.pi

    # Calculate the coordinates of the point at the desired arc length
    x_result = center[0] + 1.01 * rds * math.cos(theta_result)
    y_result = center[1] + 1.01 * rds * math.sin(theta_result)

    return (x_result, y_result)


def calculate_next_point(
    path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
):
    # Arrive end point
    if len(path_coordinate) == 1:
        return path_coordinate[0]

    tmp = calculate_distance(path_coordinate[0], path_coordinate[1])

    if (
        obstacle_or_not[0] != -1
        and obstacle_or_not[1] != -1
        and obstacle_or_not[0] == obstacle_or_not[1]
    ):  # Run in the arc path first
        if tmp < step:
            step -= tmp
            path_index.pop(0)
            path_coordinate.pop(0)
            obstacle_or_not.pop(0)
            return calculate_next_point(
                path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
            )
        else:
            return find_next_point_on_circle(
                obstacles[obstacle_or_not[0]],
                radius[obstacle_or_not[0]],
                path_coordinate[0],
                path_coordinate[1],
                step,
            )
    else:  # Run in the straight path first
        if tmp < step:
            step -= tmp
            path_index.pop(0)
            path_coordinate.pop(0)
            obstacle_or_not.pop(0)
            return calculate_next_point(
                path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
            )
        else:
            vector = (
                path_coordinate[1][0] - path_coordinate[0][0],
                path_coordinate[1][1] - path_coordinate[0][1],
            )
            magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
            vector = vector_normal(vector, magnitude, step)
            return (
                path_coordinate[0][0] + vector[0],
                path_coordinate[0][1] + vector[1],
            )


def tangent_method(obstacles, radius, s_node, e_node, step):
    # Make sure s_node isn't in any obstacle's danger area, if in, leave the obstacle first.
    tmp = point_in_obstacles(s_node, obstacles, radius)
    flag = 0
    count = 0
    while tmp != [] and count <= 5:
        flag = 1
        vector = [0.0, 0.0]
        for i in tmp:
            dis = calculate_distance(s_node, obstacles[i])
            vector[0] += (
                (s_node[0] - obstacles[i][0])
                * (1 - dis / radius[i])
                * run_distance_multiplier
            )
            vector[1] += (
                (s_node[1] - obstacles[i][1])
                * (1 - dis / radius[i])
                * run_distance_multiplier
            )

        s_node = (s_node[0] + vector[0], s_node[1] + vector[1])
        tmp = point_in_obstacles(s_node, obstacles, radius)
        count = count + 1
    if flag:
        return (s_node, [])

    # Find path
    return_main = main(obstacles, radius, s_node, e_node)

    # If no path find, don't move
    if return_main[0][0] == None or return_main[0][1] == None:
        return (s_node, [])

    path_index = return_main[0][0]  # Path with index represent
    path_coordinate = return_main[0][1]  # Path with coordinate represent
    obstacle_node = return_main[1]
    obstacle_or_not = [-1] * len(path_index)

    # Record if the point on the path is in the same obstacle as the next point or not
    for i in range(len(path_coordinate) - 1):  # Should be range(len(my_list))
        tmp = same_obstacle(
            obstacles, radius, path_coordinate[i], path_coordinate[i + 1]
        )
        if tmp != -1:
            obstacle_or_not[i] = tmp
            obstacle_or_not[i + 1] = tmp

    # Return the next point UAV move with "step" distance
    return (
        calculate_next_point(
            path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
        ),
        path_coordinate,
    )
