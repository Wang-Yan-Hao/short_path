import os
import sys
import math
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

from utility import intersect, draw_circles
from obstacle import middle_point_list, radius_list, s_start, s_goal

# Globval variable
obstacle = middle_point_list
radius = radius_list
start_node = s_start
end_node = s_goal
num = len(obstacle)
fig = plt.figure

# Graph
point_map = {start_node: 0, end_node: 1}  # 每個點對應的 index
graph = [[], []]  # 正常的 graph list, index 0 是起點, index 1 是終點
obstacle_node = [[] for _ in range(num)]  # 同一個障礙物上面的點會存在一起
point_index = 2  # 目前總共的點


def intersect_with_all_obstacles(p1, p2):  # 檢查兩點是否有跟所有障礙物接觸過
    flag = 0
    for i in range(0, num):
        ans = intersect(p1, p2, obstacle[i], radius[i])
        if ans == 1:
            flag = 1
            break
    if flag == 0:  # 沒有跟任何障礙物接觸
        plt.plot([p2[0], p1[0]], [p2[1], p1[1]], linestyle='-', color='green')
        return 1
    return 0


def find_point_cut(point, o, r):  # 找點跟圓的切線
    dis = math.sqrt((point[0] - o[0])**2 + (point[1] - o[1])**2)
    theta = math.atan((point[1] - o[1])/(point[0] - o[0]))
    theta1 = math.asin(r/dis)
    point1 = (o[0] + r*math.cos(theta + theta1 + math.pi/2),
              o[1] + r*math.sin(theta + theta1 + math.pi/2))
    point2 = (o[0] + r*math.cos(theta - theta1 - math.pi/2),
              o[1] + r*math.sin(theta - theta1 - math.pi/2))

    intersect_with_all_obstacles(point, point1)
    intersect_with_all_obstacles(point, point2)


# 畫起點跟終點
plt.scatter(start_node[0], start_node[1], color='red', marker='o')
plt.scatter(end_node[0], end_node[1], color='red', marker='o')

# 畫圓形障礙物
draw_circles(obstacle, radius)

# 找點切線
for i in range(0, num):
    find_point_cut(start_node, obstacle[i], radius[i])
    find_point_cut(end_node, obstacle[i], radius[i])

# 找 2 圓切線
for i in range(0, num):
    for j in range(i + 1, num):
        swap = 0
        # i 障礙物的半徑比 j 障礙物小
        if radius[i] > radius[j]:
            swap = 1
            i, j = j, i
        flag3 = 0
        flag4 = 0
        dis = math.sqrt((obstacle[i][0] - obstacle[j][0])
                        ** 2 + (obstacle[i][1] - obstacle[j][1])**2)
        rad_diff = radius[j] - radius[i]  # 2圓半徑差
        rad_add = radius[i] + radius[j]  # 2圓半徑和
        theta0 = math.asin(rad_diff/dis)  # 2圓間的角度
        theta1 = math.asin(rad_add/dis)
        theta2 = math.atan(
            (obstacle[j][1] - obstacle[i][1])/(obstacle[j][0] - obstacle[i][0]))  # 與水平線夾角
        # 外公切線
        theta3 = theta0 + theta2 + math.pi/2
        theta4 = theta2 - theta0 - math.pi/2
        if (obstacle[i][0] - obstacle[j][0] > 0):
            theta3 = theta3 + math.pi
            theta4 = theta4 + math.pi
        # 四個可能的切點
        point1 = (obstacle[i][0] + radius[i] * math.cos(theta3),
                  obstacle[i][1] + radius[i] * math.sin(theta3))
        point2 = (obstacle[j][0] + radius[j] * math.cos(theta3),
                  obstacle[j][1] + radius[j] * math.sin(theta3))
        point3 = (obstacle[i][0] + radius[i] * math.cos(theta4),
                  obstacle[i][1] + radius[i] * math.sin(theta4))
        point4 = (obstacle[j][0] + radius[j] * math.cos(theta4),
                  obstacle[j][1] + radius[j] * math.sin(theta4))

        intersect_with_all_obstacles(point1, point2)
        intersect_with_all_obstacles(point3, point4)

        # 內公切線
        if (dis == radius[i] + radius[j]):
            point0 = (obstacle[i][0] + radius[i] * math.cos(theta2),
                      obstacle[i][1] + radius[i] * math.sin(theta2))
            plt.scatter(point0[0], point0[1], color='red', marker='o')
        elif (dis > radius[i] + radius[j]):
            theta5 = theta2 - theta1 + math.pi/2
            theta6 = theta2 + theta1 - math.pi/2
            if (obstacle[i][0] - obstacle[j][0] > 0):
                theta5 = theta5 + math.pi
                theta6 = theta6 + math.pi
            # 四個可能的切點
            point5 = (obstacle[i][0] + radius[i] * math.cos(theta5),
                      obstacle[i][1] + radius[i] * math.sin(theta5))
            point6 = (obstacle[j][0] + radius[j] * math.cos(theta5-math.pi),
                      obstacle[j][1] + radius[j] * math.sin(theta5-math.pi))
            point7 = (obstacle[i][0] + radius[i] * math.cos(theta6),
                      obstacle[i][1] + radius[i] * math.sin(theta6))
            point8 = (obstacle[j][0] + radius[j] * math.cos(theta6-math.pi),
                      obstacle[j][1] + radius[j] * math.sin(theta6-math.pi))

            intersect_with_all_obstacles(point5, point6)
            intersect_with_all_obstacles(point7, point8)
        else:
            print('not cut line')

        if swap == 1:
            i, j = j, i

plt.xlim(0, 100)
plt.ylim(0, 100)
plt.show()
