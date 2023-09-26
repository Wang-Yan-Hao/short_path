import os
import sys
import math
import matplotlib.pyplot as plt

# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the parent directory to the Python path
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

from obstacle import radius_list, middle_point_list, x_range, y_range, s_start, s_goal

# Utility
def vector_normal(vector, magnitude, len):  # 把 vector 長度變成 len
    return (vector[0] / magnitude * len, vector[1] / magnitude * len)

def calculate_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points in a 2D space.

    Args:
    point1 (tuple): A tuple containing the coordinates of the first point (x1, y1).
    point2 (tuple): A tuple containing the coordinates of the second point (x2, y2).

    Returns:
    float: The Euclidean distance between the two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance

def are_vectors_parallel(vector1, vector2):
    # Check if the vectors are zero vectors
    if vector1 == (0, 0) or vector2 == (0, 0):
        return True  # Zero vectors are considered parallel to any vector

    # Calculate the proportionality factor for each component
    proportions = [vector1[i] / vector2[i] if vector2[i] != 0 else float('inf') for i in range(len(vector1))]

    # Check if all proportions are equal (or infinity, for zero components)
    return all(p == proportions[0] for p in proportions)

def are_points_collinear(point1, point2, point3):
    # Convert the points to vectors
    vector1 = (point2[0] - point1[0], point2[1] - point1[1])
    vector2 = (point3[0] - point1[0], point3[1] - point1[1])

    # Calculate the cross product
    cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]

    # Check if the cross product is zero (within a small tolerance)
    epsilon = 1e-6  # Tolerance for floating-point comparisons
    return abs(cross_product) < epsilon

# https://math.stackexchange.com/questions/1316803/algorithm-to-find-a-line-segment-is-passing-through-a-circle-or-not
def intersect(start_point, end_point, middle_point, radius) -> bool:  # 計算 start point 到 end point 是否有經過 middle point with radius
    print(start_point, end_point, middle_point, radius)

    if start_point ==(82.12132034355965, 77.87867965644035)and end_point == (77.87867965644035, 82.12132034355965):
        print("hjell")

    end_start = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    middle_start = (middle_point[0] - start_point[0], middle_point[1] - start_point[1])

    # Calculate the dot product of the line vector and the vector from the middle point to the start point
    dot_product = middle_start[0] * end_start[0] + middle_start[1] * end_start[1]

    # Calculate the magnitude of the line vector
    line_length = math.sqrt(end_start[0] ** 2 + end_start[1] ** 2)

    distance = abs(dot_product) / line_length

    if (middle_start[0] ** 2 + middle_start[1] ** 2 - distance ** 2) < 0 and calculate_distance(start_point, end_point) <= 2*radius+1e-6:  # 特殊情況 搞不懂
        return True
    elif (middle_start[0] ** 2 + middle_start[1] ** 2 - distance ** 2) < 0:
        return False

    h_distance = math.sqrt( (middle_start[0] ** 2 + middle_start[1] ** 2) - distance ** 2)

    # Check if the distance is less than or equal to the radius of the circle
    if h_distance < radius:
        magnitude = math.sqrt(end_start[0] ** 2 + end_start[1] ** 2)
        vertical_vector1 = vector_normal([-end_start[1], end_start[0]], magnitude, h_distance)
        vertical_vector2 = vector_normal([end_start[1], -end_start[0]], magnitude, h_distance)
        vertical_point1 = (middle_point[0] + vertical_vector1[0], middle_point[1] + vertical_vector1[1])
        vertical_point2 = (middle_point[0] + vertical_vector2[0], middle_point[1] + vertical_vector2[1])
        current_point=0
        if are_points_collinear(start_point, end_point, vertical_point1):
            current_point = vertical_point1
        else:
            current_point = vertical_point2

        if calculate_distance(start_point, current_point) > calculate_distance(start_point, end_point):
            return False
        else:
            return True
    else:
        return False

def plot_the_graph(interact_points_list):
    # Extract x and y coordinates from the list of points
    x_values, y_values = zip(*interact_points_list)

    plt.scatter(x_values, y_values, label='interact_point', color='blue', s=9, marker='o')
    plt.scatter(s_start[0], s_start[1], label='start_point', color='red', s=9, marker='o')
    plt.scatter(s_goal[0], s_goal[1], label='end_point', color='black', s=9, marker='o')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Short Path')

    # Show legend
    plt.legend(loc='upper left')

    # Create a scatter plot for points at the middle points of the circles
    plt.scatter(*zip(*middle_point_list), label='Middle Points', color='green', marker='o', s=9)

    # Plot circles based on radius_list and middle_point_list
    for i, radius in enumerate(radius_list):
        middle_point = middle_point_list[i]
        circle = plt.Circle(middle_point, radius, fill=False, color='red', linestyle='--')
        plt.gca().add_patch(circle)

    # Plot the rectangle
    rect_x = [s_start[0], s_goal[0]]
    rect_y = [s_start[1], s_goal[1]]
    plt.plot([rect_x[0], rect_x[0], rect_x[1], rect_x[1], rect_x[0]], 
             [rect_y[0], rect_y[1], rect_y[1], rect_y[0], rect_y[0]], 
             label='Rectangle', color='blue')

    # Display the plot
    # plt.show()

import heapq

def dijkstra(graph, start, end):
    # Create a priority queue (min-heap) to store nodes and their distances
    min_heap = [(0, start)]  # (distance, node)
    
    # Create a dictionary to track the shortest distances from the start node
    shortest_distances = {node: float('inf') for node in range(len(graph))}
    shortest_distances[start] = 0  # Distance from start to start is 0
    
    # Create a dictionary to store the previous node in the shortest path
    previous_nodes = {node: None for node in range(len(graph))}
    
    while min_heap:
        current_distance, current_node = heapq.heappop(min_heap)
        
        # If we have reached the end node, reconstruct and return the path
        if current_node == end:
            path = []
            while current_node is not None:
                path.insert(0, current_node)
                current_node = previous_nodes[current_node]
            return path
        
        # Skip this node if we have already found a shorter path to it
        if current_distance > shortest_distances[current_node]:
            continue
        
        # Explore neighbors
        for neighbor, edge_length in graph[current_node]:
            distance = current_distance + edge_length
            
            if distance < shortest_distances[neighbor]:
                shortest_distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(min_heap, (distance, neighbor))
    
    # If we reach this point, there is no path from start to end
    return None

def main():
    x_direction = s_goal[0] - s_start[0]
    y_direction = s_goal[1] - s_start[1]

    magnitude = math.sqrt(x_direction ** 2 + y_direction ** 2)

    points_list = [s_start, s_goal]

    for radius, middle_point in zip(radius_list, middle_point_list):
        tmp = vector_normal((x_direction, y_direction), magnitude, radius)
        tmp_point = (middle_point[0] + tmp[0], middle_point[1] + tmp[1])
        points_list.append(tmp_point)

        tmp =  vector_normal((-x_direction, -y_direction), magnitude, radius)
        tmp_point = (middle_point[0] + tmp[0], middle_point[1] + tmp[1])
        points_list.append(tmp_point)

        tmp =  vector_normal((y_direction, -x_direction), magnitude, radius)
        tmp_point = (middle_point[0] + tmp[0], middle_point[1] + tmp[1])
        points_list.append(tmp_point)

        tmp =  vector_normal((-y_direction, x_direction), magnitude, radius)
        tmp_point = (middle_point[0] + tmp[0], middle_point[1] + tmp[1])
        points_list.append(tmp_point)

    # Call the plot function to visualize the points and shapes
    plot_the_graph(points_list)

    # Create a list of lists of length len(points_list)
    graph = [[] for _ in range(len(points_list))]
    points_list_len = len(points_list)

    index_line = 0
    index_not_line = 0

    for i in range(0, points_list_len-1):
        for j in range(i+1, points_list_len):

            current_point = points_list[i]
            end_point = points_list[j]
            print("新的一輪")
            flag = 0
            for k in range(0, len(middle_point_list)):  # 查看每個障礙物，有沒有障礙物擋在 i 跟 j 中間
                if intersect(current_point, end_point, middle_point_list[k], radius_list[k]): # 如果有障礙物
                    print("有障礙物")
                    index_not_line = index_not_line + 1
                    flag = 1
                    break
            
            if not flag:
                print("沒有障礙物")
                index_line = index_line + 1
                i_j_len = calculate_distance(current_point, end_point)
                graph[i].append((j, i_j_len))  # point_index and the len of the point
                graph[j].append((i, i_j_len))
                plt.plot([current_point[0], end_point[0]], [current_point[1], end_point[1]], linestyle='-', color='green', label='Line')

    
    print(index_line)
    print(index_not_line)
    plt.show()
    plt.pause(100)
    print(dijkstra(graph, 0, 1))

if __name__ == '__main__':
    main()