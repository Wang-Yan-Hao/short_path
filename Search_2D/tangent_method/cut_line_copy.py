import heapq
import math
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, "..")
sys.path.append(parent_dir)

# from obstacle import middle_point_list, radius_list, s_goal, s_start
from utility import (
    calculate_distance,
    calculate_two_point_distance_in_circle,
    draw_circles,
    intersect,
)


def intersect_with_all_obstacles(
    p1, p2, not_include_p1=False, p1_obstacle_index=-1, p2_obstacle_index=-1
):  # 檢查兩點是否有跟所有障礙物接觸過
    global point_index
    flag = 0
    for i in range(0, obstacle_num):
        if i == p2_obstacle_index:
            continue
        ans = intersect(p1, p2, obstacle[i], radius[i])

        # 要跳過 p1 p2 所在的障礙物
        if not_include_p1 and i == p2_obstacle_index:
            continue
        elif i == p1_obstacle_index or i == p2_obstacle_index:
            continue
        if (
            calculate_distance(p1, obstacle[i]) < radius[i]
            or calculate_distance(p2, obstacle[i]) < radius[i]
        ):
            flag = 1
            break
        if ans == 1:
            flag = 1
            break

    if flag == 0:  # 沒有跟任何障礙物接觸
        if not_include_p1:  # p1 是起點或者終點, 所以 p1 不用被 obstacle_node append
            path_len = calculate_distance(p1, p2)

            # 為新加的點設個對應的 index
            if p1 not in point_map:
                point_map[p1] = point_index
                point_index = point_index + 1
                graph.append([])  # graph 要為新的點加一個 list
            if p2 not in point_map:
                point_map[p2] = point_index
                point_index = point_index + 1
                graph.append([])  # graph 要為新的點加一個 list

            p1_index = point_map[p1]
            p2_index = point_map[p2]

            graph[p1_index].append((p2_index, path_len))  # p1 加上 p2
            graph[p2_index].append((p1_index, path_len))  # p2 加上 p1
            obstacle_node[p2_obstacle_index].append(
                p2
            )  # 第 p2_obstacle_index 個 obstacle 要加上 p2
        else:
            path_len = calculate_distance(p1, p2)

            # 新加的點設個對應的 index
            if p1 not in point_map:
                point_map[p1] = point_index
                point_index = point_index + 1
                graph.append([])  # graph 要為新的點加一個 list
            if p2 not in point_map:
                point_map[p2] = point_index
                point_index = point_index + 1
                graph.append([])  # graph 要為新的點加一個 list

            p1_index = point_map[p1]
            p2_index = point_map[p2]

            graph[p1_index].append((p2_index, path_len))  # p1 加上 p2
            graph[p2_index].append((p1_index, path_len))  # p2 加上 p1
            obstacle_node[p1_obstacle_index].append(
                p1
            )  # 第 p1_obstacle_index 個 obstacle 要加上 p1
            obstacle_node[p2_obstacle_index].append(
                p2
            )  # 第 p2_obstacle_index 個 obstacle 要加上 p2
        return 1
    return 0


# 找點跟圓的切線
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
    # print(point,point1,point2)
    intersect_with_all_obstacles(
        point, point1, not_include_p1=True, p2_obstacle_index=obstacle_index
    )
    intersect_with_all_obstacles(
        point, point2, not_include_p1=True, p2_obstacle_index=obstacle_index
    )


# 圓跟圓的外切點
def find_outer_cut(pointi, pointj, radiusi, radiusj, dis, ob_i, ob_j):
    rad_diff = radiusj - radiusi
    if pointj[0] - pointi[0] == 0:
        theta0 = math.pi / 2
    else:
        theta0 = math.atan((pointj[1] - pointi[1]) / (pointj[0] - pointi[0]))  # 與水平線夾角
    if dis > rad_diff:
        theta1 = math.asin(rad_diff / dis)  # 2圓間的角度
        theta2 = theta1 + theta0 + math.pi / 2
        theta3 = theta0 - theta1 - math.pi / 2
        if pointi[0] - pointj[0] > 0:
            theta2 = theta2 + math.pi
            theta3 = theta3 + math.pi
        # 四個可能的切點
        point1 = (
            pointi[0] + radiusi * math.cos(theta2),
            pointi[1] + radiusi * math.sin(theta2),
        )
        point2 = (
            pointj[0] + radiusj * math.cos(theta2),
            pointj[1] + radiusj * math.sin(theta2),
        )
        point3 = (
            pointi[0] + radiusi * math.cos(theta3),
            pointi[1] + radiusi * math.sin(theta3),
        )
        point4 = (
            pointj[0] + radiusj * math.cos(theta3),
            pointj[1] + radiusj * math.sin(theta3),
        )
        intersect_with_all_obstacles(point1, point2, False, ob_i, ob_j)
        intersect_with_all_obstacles(point3, point4, False, ob_i, ob_j)
    else:
        pass


# 圓跟圓的內切點
def find_inner_cut(pointi, pointj, radiusi, radiusj, dis, ob_i, ob_j):
    rad_add = radiusi + radiusj  # 2圓半徑和
    if pointj[0] - pointi[0] == 0:
        theta0 = math.pi / 2
    else:
        theta0 = math.atan((pointj[1] - pointi[1]) / (pointj[0] - pointi[0]))  # 與水平線夾角
    if dis > radiusi + radiusj:
        theta1 = math.asin(rad_add / dis)
        theta2 = theta0 - theta1 + math.pi / 2
        theta3 = theta0 + theta1 - math.pi / 2
        if pointi[0] - pointj[0] > 0:
            theta2 = theta2 + math.pi
            theta3 = theta3 + math.pi
        # 四個可能的切點
        point5 = (
            pointi[0] + radiusi * math.cos(theta2),
            pointi[1] + radiusi * math.sin(theta2),
        )
        point6 = (
            pointj[0] + radiusj * math.cos(theta2 - math.pi),
            pointj[1] + radiusj * math.sin(theta2 - math.pi),
        )
        point7 = (
            pointi[0] + radiusi * math.cos(theta3),
            pointi[1] + radiusi * math.sin(theta3),
        )
        point8 = (
            pointj[0] + radiusj * math.cos(theta3 - math.pi),
            pointj[1] + radiusj * math.sin(theta3 - math.pi),
        )

        intersect_with_all_obstacles(point5, point6, False, ob_i, ob_j)
        intersect_with_all_obstacles(point7, point8, False, ob_i, ob_j)
    else:
        pass


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
                graph[p1_index].append((p2_index, path_len))  # p1 加上 p2
                graph[p2_index].append((p1_index, path_len))  # p2 加上 p1


def dijkstra(graph, start_node, end_node):
    num_nodes = len(graph)  # Assuming graph is a list of nodes and their connections

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
    path = []  # 最短路徑, 用 index 表示
    current_node = end_node
    while current_node != -1:
        path.insert(0, current_node)  # Insert at the beginning to reverse the path
        current_node = parent[current_node]

    # 最短路徑, 用作標點表示
    shortest_path = [
        point
        for index in path
        for point, point_index in point_map.items()
        if point_index == index
    ]
    # Plot the path by connecting the points
    # path_x = [point[0] for point in shortest_path]
    # path_y = [point[1] for point in shortest_path]

    # plt.show()
    return (path, shortest_path)


def main(obstacles, r, s_node, e_node):
    # 重製全域變數
    global point_map, graph, point_index, obstacle_node, obstacle, radius, start_node, end_node, obstacle_num
    obstacle = obstacles
    radius = r
    start_node = s_node
    end_node = e_node
    obstacle_num = len(obstacle)

    point_map = {start_node: 0, end_node: 1}  # 每個點對應的 index
    graph = [[], []]  # 正常的 graph list, index 0 是起點, index 1 是終點
    point_index = 2  # 目前總共的點
    obstacle_node = [[] for _ in range(obstacle_num)]  # 同一個障礙物上面的點會存在一起

    # 畫圓形障礙物
    # draw_circles(obstacle, radius)

    # 找點切線
    for i in range(0, obstacle_num):
        find_point_cut(start_node, obstacle[i], radius[i], i)
        find_point_cut(end_node, obstacle[i], radius[i], i)

    # 找 2 圓切線
    for i in range(0, obstacle_num):
        for j in range(i + 1, obstacle_num):
            swap = 0
            # i 障礙物的半徑比 j 障礙物小
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

    
    # 同一個障礙物上面的點要連線
    obstacle_node_add_to_graph()

    intersect_with_all_obstacles(start_node, end_node)
    
    return [dijkstra(graph, 0, 1), obstacle_node]
