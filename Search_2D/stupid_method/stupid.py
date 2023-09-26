import os
import sys
import math
import matplotlib.pyplot as plt

# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the parent directory to the Python path
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

from obstacle import radius_list, middle_point_list, s_start, s_goal

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