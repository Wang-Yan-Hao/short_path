import matplotlib.pyplot as plt
import heapq
import math


def vector_normal(vector, magnitude, len):
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
    proportions = [vector1[i] / vector2[i] if vector2[i] !=
                   0 else float('inf') for i in range(len(vector1))]

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
def intersect(p1, p2, o, r) -> bool:  # 計算 start point 到 end point 是否有經過 middle point with radius
    v = (p2[0]-p1[0], p2[1]-p1[1])
    # (p1[0] + t*v[0] - obstacle[0][0])**2 + (p1[1] + t*v[1] - obstacle[0][1])**2 = radius[0]**2
    # (v[0]**2 + v[1]**2)*t^2 + 2*(p1[0]*v[0] - v[0]*obstacle[0][0] + p1[1]*v[1] - v[1]*obstacle[0][1])*t +
    # (p1[0]**2 + obstacle[0][0]**2 + p1[1]**2 + obstacle[0][1]**2 - radius[0]**2 - 2*p1[0]*obstacle[0][0] - 2*p1[1]*obstacle[0][1]) = 0
    a = v[0]**2 + v[1]**2
    b = 2*(p1[0]*v[0] - v[0]*o[0] + p1[1]*v[1] - v[1]*o[1])
    c = p1[0]**2 + o[0]**2 + p1[1]**2 + o[1]**2 - \
        r**2 - 2*p1[0]*o[0] - 2*p1[1]*o[1]
    D = b**2 - 4*a*c
    if D > 0:
        t1 = (-b + math.sqrt(D))/(2*a)
        t2 = (-b - math.sqrt(D))/(2*a)
        # print(t1,t2)
        K = (p1[0] + t1*v[0], p1[1] + t1*v[1])
        K1 = (p1[0] + t2*v[0], p1[1] + t2*v[1])
        # plt.scatter(K[0], K[1], color='blue', marker='o')
        # plt.scatter(K1[0], K1[1], color='blue', marker='o')
        if ((t1 >= 0) and (t1 <= 1)) or ((t2 >= 0) and (t2 <= 1)):
            return 1
    # print(a,b,c,D)
    # plt.show()
    return 0

# def plot_the_graph(interact_points_list):
#     # Extract x and y coordinates from the list of points
#     x_values, y_values = zip(*interact_points_list)

#     plt.scatter(x_values, y_values, label='interact_point', color='blue', s=9, marker='o')
#     plt.scatter(s_start[0], s_start[1], label='start_point', color='red', s=9, marker='o')
#     plt.scatter(s_goal[0], s_goal[1], label='end_point', color='black', s=9, marker='o')

#     # Add labels and title
#     plt.xlabel('X-axis')
#     plt.ylabel('Y-axis')
#     plt.title('Short Path')

#     # Show legend
#     plt.legend(loc='upper left')

#     # Create a scatter plot for points at the middle points of the circles
#     plt.scatter(*zip(*middle_point_list), label='Middle Points', color='green', marker='o', s=9)

#     # Plot circles based on radius_list and middle_point_list
#     for i, radius in enumerate(radius_list):
#         middle_point = middle_point_list[i]
#         circle = plt.Circle(middle_point, radius, fill=False, color='red', linestyle='--')
#         plt.gca().add_patch(circle)

#     # Plot the rectangle
#     rect_x = [s_start[0], s_goal[0]]
#     rect_y = [s_start[1], s_goal[1]]
#     plt.plot([rect_x[0], rect_x[0], rect_x[1], rect_x[1], rect_x[0]],
#              [rect_y[0], rect_y[1], rect_y[1], rect_y[0], rect_y[0]],
#              label='Rectangle', color='blue')
    # Display the plot
    # plt.show()


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


# 畫圓形障礙物
def draw_circles(obstacle, radius):
    for i in range(0, len(obstacle)):
        circle = plt.Circle(obstacle[i], radius[i], fill=False)
        plt.gca().add_patch(circle)


def calculate_two_point_distance_in_circle(p1, p2, center, radius):
    # Calculate the angles formed by the two points at the center
    angle1 = math.atan2(p1[1] - center[1], p1[0] - center[0])
    angle2 = math.atan2(p2[1] - center[1], p2[0] - center[0])
    
    # Ensure angles are between 0 and 2*pi
    if angle1 < 0:
        angle1 += 2 * math.pi
    if angle2 < 0:
        angle2 += 2 * math.pi
    
    # Calculate the arc length between the two points
    arc_length = abs(angle2 - angle1) * radius

    return arc_length