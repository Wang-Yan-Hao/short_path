import math
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, "..")
sys.path.append(parent_dir)
parent_dir = os.path.join(current_dir, ".")
sys.path.append(parent_dir)

from cut_line_copy import main
from utility import calculate_distance, vector_normal

step = 15


def same_obstacle(obstacle_node, s, e):
    flag_1 = 0
    flag_2 = 0
    for i, list in enumerate(obstacle_node):
        for element in list:
            if element == s:
                flag_1 = 1
            elif element == e:
                flag_2 = 1
        if flag_1 and flag_2:
            return i
        elif flag_1 or flag_2:
            return -1
    return -1


def cartesian_to_polar(x, y, circle_center):
    x0, y0 = circle_center
    theta = math.atan2(y - y0, x - x0)
    return theta


def find_poiont_on_circle(center, r, point_1, point_2, length):
    # Convert Cartesian coordinates to polar coordinates
    theta1 = cartesian_to_polar(point_1[0], point_1[1], center)
    theta2 = cartesian_to_polar(point_2[0], point_2[1], center)

    # Calculate the angular difference between the two points
    delta_theta = theta2 - theta1

    # 圓周 英文
    circumference = r * 2 * math.pi

    theta_result = theta1
    if delta_theta < 0:
        # Calculate the angle of the point on the arc
        theta_result -= length / circumference * 2 * math.pi
    else:
        theta_result += length / circumference * 2 * math.pi

    # Calculate the coordinates of the point at the desired arc length
    x_result = center[0] + r * math.cos(theta_result)
    y_result = center[1] + r * math.sin(theta_result)

    return (x_result, y_result)


def calculate_next_point(
    path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
):
    tmp = calculate_distance(path_coordinate[0], path_coordinate[1])

    if (
        obstacle_or_not[0] != -1
        and obstacle_or_not[1] != -1
        and obstacle_or_not[0] == obstacle_or_not[1]
    ):  # 先走弧線
        if tmp < step:
            step -= tmp
            path_index.pop(0)
            path_coordinate.pop(0)
            obstacle_or_not.pop(0)
            return calculate_next_point(
                path_index, path_coordinate, obstacle_or_not, step, obstacles, radius
            )
        else:
            return find_poiont_on_circle(
                obstacles[obstacle_or_not[0]],
                radius[obstacle_or_not[0]],
                path_coordinate[0],
                path_coordinate[1],
                step,
            )
    else:  # 先走直線
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


def tangent_method(obstacles, r, s_node, e_node):
    # 因為他們座標跟我們不一樣
    # s_node = (1700 - s_node[1], 1250 - s_node[0])
    # e_node = (1700 - e_node[1], 1250 - e_node[0])
    # for i in range(len(obstacles)):
    #     obstacles[i] = (1700 - obstacles[i][1], 1250 - obstacles[i][0])

    # 先判斷起點有沒有在任意障礙物裡面
    for i in range(len(obstacles)):
        tmp = calculate_distance(obstacles[i], s_node)
        # 如果在障礙物裡面，直接跳到圓周上, todo: 之後在改成走 step 距離
        if tmp < r[i]:
            vector = (s_node[0] - obstacles[i][0], s_node[1] - obstacles[i][1])
            vector = (
                vector[0] * (1 - tmp / r[i]) * 5,
                vector[1] * (1 - tmp / r[i]) * 5,
            )
            s_node = (s_node[0] + vector[0], s_node[1] + vector[1])
            return (s_node, [])
    # main algorithm
    return_main = main(obstacles, r, s_node, e_node)

    # 如果找不到路徑 就不動
    if return_main[0][0] == None or return_main[0][1] == None:
        return (s_node, [])

    path_index = return_main[0][0]  # 用 index 表示的 path
    path_coordinate = return_main[0][1]  # 用座標點表示的 path
    obstacle_node = return_main[1]
    obstacle_or_not = [-1] * len(path_index)

    # 用 obstacle_or_not 紀錄這些點是否在同一障礙物上面
    for i in range(len(path_index) - 1):  # Should be range(len(my_list))
        tmp = same_obstacle(obstacle_node, path_coordinate[i], path_coordinate[i + 1])
        if tmp != -1:
            obstacle_or_not[i] = tmp
            obstacle_or_not[i + 1] = tmp

    # 回傳下一個所在的點, 每一 frame 走長度 step
    return (
        calculate_next_point(
            path_index, path_coordinate, obstacle_or_not, step, obstacles, r
        ),
        path_coordinate,
    )
