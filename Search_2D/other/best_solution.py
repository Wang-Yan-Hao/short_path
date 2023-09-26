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

def intersect(start_point, end_point, middle_point, radius) -> bool:
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
    plt.legend()

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
    # 1 mean right or top direction
    x_direction = 1 if s_goal[0] - s_start[0] > 0 else -1
    y_direction = 1 if s_goal[1] - s_start[1] > 0 else -1

    # x_direction =  1 y_direction =  1  mean sort the point x ascending then y ascending
    # x_direction =  1 y_direction = -1  mean sort the point x ascending then y descending
    # x_direction = -1 y_direction =  1  mean sort the point x descneding then y ascending
    # x_direction = -1 y_direction = -1  mean sort the point x descneding then y descending

    # Define a custom sorting key function for both middle_point_list and radius_list
    def custom_sort(point_radius_pair):
        middle_point, radius = point_radius_pair
        x, y = middle_point
        return (x_direction * x, y_direction * y)

    # Combine middle_point_list and radius_list into pairs and sort them together
    combined_list = list(zip(middle_point_list, radius_list))
    sorted_combined_list = sorted(combined_list, key=custom_sort)

    # Unpack the sorted values into separate lists
    sorted_middle_point_list, sorted_radius_list = zip(*sorted_combined_list)
    point_numbers = len(sorted_middle_point_list)

    interact_points_list = [s_start]
    s_start_buffer = [s_start]

    for i in range(0, point_numbers):
        buffer_len = len(s_start_buffer)
        counter = 0
        for s_start_in_buffer in s_start_buffer:
            counter = counter + 1
            interact_points = intersect(s_start_in_buffer, s_goal, sorted_middle_point_list[i], sorted_radius_list[i])
            
            if interact_points:
                s_start_buffer.remove(s_start_in_buffer)

                if interact_points[0]:
                    s_start_buffer.append(interact_points[0])
                    interact_points_list.append(interact_points[0])
                if interact_points[1]:
                    s_start_buffer.append(interact_points[1])
                    interact_points_list.append(interact_points[1])

            if counter == buffer_len:
                break
    plot_the_graph(interact_points_list)

if __name__ == '__main__':
    main()