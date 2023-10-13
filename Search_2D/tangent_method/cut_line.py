import heapq
import math
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, "..")
sys.path.append(parent_dir)

from utility import (
    calculate_distance,
    calculate_two_point_distance_in_circle,
    intersect,
)


def intersect_with_all_obstacles(
    p1, p2, not_include_p1=False, p1_obstacle_index=-1, p2_obstacle_index=-1
):  # Check whether the two points have been in contact with all obstacles
    global point_index
    flag = 1
    for i in range(obstacle_num):
        if i == p2_obstacle_index:
            continue
        ans = intersect(p1, p2, obstacle[i], radius[i])

        # To skip the obstacle where p1 p2 is located
        if not_include_p1 and i == p2_obstacle_index:
            continue
        elif i == p1_obstacle_index or i == p2_obstacle_index:
            continue
        if (
            calculate_distance(p1, obstacle[i]) < radius[i]
            or calculate_distance(p2, obstacle[i]) < radius[i]
        ):
            flag = 0
            break
        if ans == 1:
            flag = 0
            break

    if flag:  # No contact with any obstacles
        if (
            not_include_p1
        ):  # p1 is the starting point or end point, so p1 does not need to be appended by obstacle_node
            path_len = calculate_distance(p1, p2)

            # Set a corresponding index for the newly added point
            if p1 not in point_map:
                point_map[p1] = point_index
                point_index = point_index + 1
                graph.append([])
            if p2 not in point_map:
                point_map[p2] = point_index
                point_index = point_index + 1
                graph.append([])

            p1_index = point_map[p1]
            p2_index = point_map[p2]

            graph[p1_index].append((p2_index, path_len))
            graph[p2_index].append((p1_index, path_len))
            obstacle_node[p2_obstacle_index].append(p2)
        else:
            path_len = calculate_distance(p1, p2)

            if p1 not in point_map:
                point_map[p1] = point_index
                point_index = point_index + 1
                graph.append([])
            if p2 not in point_map:
                point_map[p2] = point_index
                point_index = point_index + 1
                graph.append([])

            p1_index = point_map[p1]
            p2_index = point_map[p2]

            graph[p1_index].append((p2_index, path_len))
            graph[p2_index].append((p1_index, path_len))
            obstacle_node[p1_obstacle_index].append(p1)
            obstacle_node[p2_obstacle_index].append(p2)
        return 1
    return 0


# Find the tangent line between the point and the circle
def find_point_cut(point, o, r, obstacle_index):
    dis = math.sqrt((point[0] - o[0]) ** 2 + (point[1] - o[1]) ** 2)
    theta = math.atan((point[1] - o[1]) / (point[0] - o[0]))
    theta1 = math.asin(r / dis)
    if point[0] > o[0]:
        point1 = (
            o[0] + r * math.cos(theta - theta1 + math.pi / 2),
            o[1] + r * math.sin(theta - theta1 + math.pi / 2),
        )
        point2 = (
            o[0] + r * math.cos(theta + theta1 - math.pi / 2),
            o[1] + r * math.sin(theta + theta1 - math.pi / 2),
        )
    else:
        point1 = (
            o[0] + r * math.cos(theta + theta1 + math.pi / 2),
            o[1] + r * math.sin(theta + theta1 + math.pi / 2),
        )
        point2 = (
            o[0] + r * math.cos(theta - theta1 - math.pi / 2),
            o[1] + r * math.sin(theta - theta1 - math.pi / 2),
        )
    intersect_with_all_obstacles(
        point, point1, not_include_p1=True, p2_obstacle_index=obstacle_index
    )
    intersect_with_all_obstacles(
        point, point2, not_include_p1=True, p2_obstacle_index=obstacle_index
    )


# Find the circumscribed points of a circle and a circle
def find_outer_cut(p_i, p_j, rds_i, rds_j, dis, ob_i, ob_j):
    rad_diff = rds_j - rds_i
    if p_j[0] - p_i[0] == 0:
        theta0 = math.pi / 2
    else:
        theta0 = math.atan(
            (p_j[1] - p_i[1]) / (p_j[0] - p_i[0])
        )  # Angle with horizontal line
    if dis > rad_diff:
        theta1 = math.asin(rad_diff / dis)  # Angle between two circles
        theta2 = theta1 + theta0 + math.pi / 2
        theta3 = theta0 - theta1 - math.pi / 2
        if p_i[0] - p_j[0] > 0:
            theta2 = theta2 + math.pi
            theta3 = theta3 + math.pi
        # Four possible tangent points
        point1 = (
            p_i[0] + rds_i * math.cos(theta2),
            p_i[1] + rds_i * math.sin(theta2),
        )
        point2 = (
            p_j[0] + rds_j * math.cos(theta2),
            p_j[1] + rds_j * math.sin(theta2),
        )
        point3 = (
            p_i[0] + rds_i * math.cos(theta3),
            p_i[1] + rds_i * math.sin(theta3),
        )
        point4 = (
            p_j[0] + rds_j * math.cos(theta3),
            p_j[1] + rds_j * math.sin(theta3),
        )
        intersect_with_all_obstacles(point1, point2, False, ob_i, ob_j)
        intersect_with_all_obstacles(point3, point4, False, ob_i, ob_j)
    else:
        pass


# Find the inscribed points of a circle and a circle
def find_inner_cut(p_i, p_j, rds_i, rds_j, dis, ob_i, ob_j):
    rad_add = rds_i + rds_j  # two circle radius sum
    if p_j[0] - p_i[0] == 0:
        theta0 = math.pi / 2
    else:
        theta0 = math.atan((p_j[1] - p_i[1]) / (p_j[0] - p_i[0]))  # 與水平線夾角
    if dis > rds_i + rds_j:
        theta1 = math.asin(rad_add / dis)
        theta2 = theta0 - theta1 + math.pi / 2
        theta3 = theta0 + theta1 - math.pi / 2
        if p_i[0] - p_j[0] > 0:
            theta2 = theta2 + math.pi
            theta3 = theta3 + math.pi
        # Four possible tangent points
        point5 = (
            p_i[0] + rds_i * math.cos(theta2),
            p_i[1] + rds_i * math.sin(theta2),
        )
        point6 = (
            p_j[0] + rds_j * math.cos(theta2 - math.pi),
            p_j[1] + rds_j * math.sin(theta2 - math.pi),
        )
        point7 = (
            p_i[0] + rds_i * math.cos(theta3),
            p_i[1] + rds_i * math.sin(theta3),
        )
        point8 = (
            p_j[0] + rds_j * math.cos(theta3 - math.pi),
            p_j[1] + rds_j * math.sin(theta3 - math.pi),
        )

        intersect_with_all_obstacles(point5, point6, False, ob_i, ob_j)
        intersect_with_all_obstacles(point7, point8, False, ob_i, ob_j)
    else:
        pass


# All points in same obstacle sholud add a path(arc path)
def obstacle_node_add_to_graph():
    for index, obstacle_points in enumerate(obstacle_node):
        points_obstacle_num = len(obstacle_points)
        for i in range(0, points_obstacle_num):
            for j in range(0, points_obstacle_num):
                if j == i:
                    continue
                p1 = obstacle_points[i]
                p2 = obstacle_points[j]
                path_len = calculate_two_point_distance_in_circle(
                    p1, p2, obstacle[index], radius[index]
                )
                p1_index = point_map[p1]
                p2_index = point_map[p2]
                graph[p1_index].append((p2_index, path_len))
                graph[p2_index].append((p1_index, path_len))


def dijkstra(graph, start_node, end_node):
    num_nodes = len(graph)

    # Initialize distances and parent arrays
    distances = [float("inf")] * num_nodes
    parent = [-1] * num_nodes

    # Set the distance of the start node to 0
    start_index = start_node
    distances[start_index] = 0

    # Create a priority queue (heap) to store vertices and their distances
    priority_queue = [(0, start_index)]

    while priority_queue:
        # Get the vertex with the minimum distance
        if not priority_queue:
            break  # If the priority queue is empty, exit the loop
        dist_u, u = heapq.heappop(priority_queue)

        # If the vertex has already been visited, skip it
        if dist_u > distances[u]:
            continue

        # Update the distances to the neighboring vertices
        for v, weight in graph[u]:
            if distances[u] + weight < distances[v]:
                distances[v] = distances[u] + weight
                parent[v] = u  # Record the parent of node v
                heapq.heappush(priority_queue, (distances[v], v))

    # Check if there is no valid path to the end_node
    if distances[end_node] == float("inf"):
        return (None, None)  # Return None to indicate no valid path

    # Reconstruct the shortest path using the parent array
    path = []  # Shortest path with point index to represent
    current_node = end_node
    while current_node != -1:
        path.insert(0, current_node)  # Insert at the beginning to reverse the path
        current_node = parent[current_node]

    # Shortest path with point coordinate to represent
    shortest_path = [
        point
        for index in path
        for point, point_index in point_map.items()
        if point_index == index
    ]

    return (path, shortest_path)


def main(obstacles, r, s_node, e_node):
    # 重製全域變數
    global point_map, graph, point_index, obstacle_node, obstacle, radius, start_node, end_node, obstacle_num
    obstacle = obstacles
    radius = r
    start_node = s_node
    end_node = e_node
    obstacle_num = len(obstacle)

    point_map = {start_node: 0, end_node: 1}  # The index corresponding to each point
    graph = [[], []]  # Edge list
    point_index = 2  # Total point
    obstacle_node = [
        [] for _ in range(obstacle_num)
    ]  # Points on the same obstacle will exist together, in same list

    # If there is a path from start to end, return the path directly
    if intersect_with_all_obstacles(start_node, end_node):
        return [dijkstra(graph, 0, 1), obstacle_node]

    # Find tangent point between start point to all obstacles and end point to all obstacles
    for i in range(0, obstacle_num):
        find_point_cut(start_node, obstacle[i], radius[i], i)
        find_point_cut(end_node, obstacle[i], radius[i], i)

    # Find tangent point between any two obstacles
    for i in range(0, obstacle_num):
        for j in range(i + 1, obstacle_num):
            swap = 0
            if radius[i] > radius[j]:
                swap = 1
                i, j = j, i
            dis = math.sqrt(
                (obstacle[i][0] - obstacle[j][0]) ** 2
                + (obstacle[i][1] - obstacle[j][1]) ** 2
            )
            find_outer_cut(obstacle[i], obstacle[j], radius[i], radius[j], dis, i, j)
            find_inner_cut(obstacle[i], obstacle[j], radius[i], radius[j], dis, i, j)
            if swap == 1:
                i, j = j, i

    # Points on same obstacles may have a path to each other
    obstacle_node_add_to_graph()

    return [dijkstra(graph, 0, 1), obstacle_node]
