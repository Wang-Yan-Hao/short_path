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

def intersect(start_point, end_point, middle_point, radius) -> bool:  # 計算 start point 到 end point 是否有經過 middle point with radius
    # Calculate the direction vector of the line segment
    line_vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])

    # Calculate the vector from the middle point of the circle to the start point of the line segment
    middle_to_start = (start_point[0] - middle_point[0], start_point[1] - middle_point[1])

    # Calculate the dot product of the line vector and the vector from the middle point to the start point
    dot_product = middle_to_start[0] * line_vector[0] + middle_to_start[1] * line_vector[1]

    # Calculate the magnitude of the line vector
    line_length = math.sqrt(line_vector[0] ** 2 + line_vector[1] ** 2)

    distance = abs(dot_product) / line_length
    h_distance = math.sqrt(middle_to_start[0] ** 2 + middle_to_start[1] ** 2 - distance ** 2)

    # Check if the distance is less than or equal to the radius of the circle
    if h_distance <= radius:
        # Calculate the magnitude (length) of the vector
        magnitude = math.sqrt(line_vector[0] ** 2 + line_vector[1] ** 2)

        # Calculate the clockwise normal vector
        normal_clockwise = vector_normal((line_vector[1], -line_vector[0]), magnitude, radius)

        # Calculate the counterclockwise normal vector
        normal_counterclockwise =  vector_normal((-line_vector[1], line_vector[0]), magnitude, radius)

        # Calculate the intersection points
        intersection1 = (middle_point[0] + normal_clockwise[0], middle_point[1] + normal_clockwise[1])
        intersection2 = (middle_point[0] + normal_counterclockwise[0], middle_point[1] + normal_counterclockwise[1])

        if intersection1[0] < 0 or intersection1[0] > x_range or intersection1[1] < 0 or intersection1[1] > y_range:
            intersection1 = None
        if intersection2[0] < 0 or intersection2[0] > x_range or intersection2[1] < 0 or intersection2[1] > y_range:
            intersection2 = None
        
        return intersection1, intersection2
    else:
        # Return None if there is no intersection
        return None

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
    plt.show()
    plt.pause(100)

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

    # for i in range(0, len(points_list)-1):
    #     for j in range(0, len(points_list)):


if __name__ == '__main__':
    main()