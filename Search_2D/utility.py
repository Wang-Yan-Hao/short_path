import math

import matplotlib.pyplot as plt


# Euclidean distance
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


def intersect(
    p1, p2, o, r
) -> (
    bool
):  # Calculate whether start point to end point passes through center point with radius
    v = (p2[0] - p1[0], p2[1] - p1[1])
    # (p1[0] + t*v[0] - obstacle[0][0])**2 + (p1[1] + t*v[1] - obstacle[0][1])**2 = radius[0]**2
    # (v[0]**2 + v[1]**2)*t^2 + 2*(p1[0]*v[0] - v[0]*obstacle[0][0] + p1[1]*v[1] - v[1]*obstacle[0][1])*t +
    # (p1[0]**2 + obstacle[0][0]**2 + p1[1]**2 + obstacle[0][1]**2 - radius[0]**2 - 2*p1[0]*obstacle[0][0] - 2*p1[1]*obstacle[0][1]) = 0
    a = v[0] ** 2 + v[1] ** 2
    b = 2 * (p1[0] * v[0] - v[0] * o[0] + p1[1] * v[1] - v[1] * o[1])
    c = (
        p1[0] ** 2
        + o[0] ** 2
        + p1[1] ** 2
        + o[1] ** 2
        - r**2
        - 2 * p1[0] * o[0]
        - 2 * p1[1] * o[1]
    )
    D = b**2 - 4 * a * c
    if D > 0:
        t1 = (-b + math.sqrt(D)) / (2 * a)
        t2 = (-b - math.sqrt(D)) / (2 * a)
        # print(t1,t2)
        K = (p1[0] + t1 * v[0], p1[1] + t1 * v[1])
        K1 = (p1[0] + t2 * v[0], p1[1] + t2 * v[1])
        # plt.scatter(K[0], K[1], color='blue', marker='o')
        # plt.scatter(K1[0], K1[1], color='blue', marker='o')
        if ((t1 >= 0) and (t1 <= 1)) or ((t2 >= 0) and (t2 <= 1)):
            return 1
    # print(a,b,c,D)
    # plt.show()
    return 0


# Draw
def draw_circles(obstacle, radius):
    for i in range(0, len(obstacle)):
        circle = plt.Circle(obstacle[i], radius[i], fill=False)
        plt.gca().add_patch(circle)


# Calculate the arc length between the two points
def calculate_two_point_distance_in_circle(p1, p2, center, radius):
    # Calculate the angles formed by the two points at the center
    angle1 = math.atan2(p1[1] - center[1], p1[0] - center[0])
    angle2 = math.atan2(p2[1] - center[1], p2[0] - center[0])

    current_angle = angle2 - angle1
    if current_angle < 0:
        current_angle += 2 * math.pi
    if current_angle > math.pi:
        current_angle = 2 * math.pi - current_angle

    arc_length = current_angle * radius

    return arc_length


# Check what obs include the p, return a list to represent all obs index
def point_in_obstacles(p, obs, rds):
    output = []
    for i in range(len(obs)):
        tmp = calculate_distance(p, obs[i])
        if tmp < rds[i]:
            output.append(i)
    return output
