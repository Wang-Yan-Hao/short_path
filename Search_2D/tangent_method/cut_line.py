import matplotlib.pyplot as plt

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, '..')
sys.path.append(parent_dir)

from utility import intersect, calculate_distance
import numpy as np
import math

from obstacle import middle_point_list, radius_list, s_start, s_goal
obstacle = middle_point_list
radius = radius_list
start_node = s_start
end_node = s_goal
# obstacle = [(56,30),(62,38),(74,28),(37,99),(55,11),(67,12),(43,44),(78,71),(22,20)]
# radius = [5,5,4,7,6,3,8,2,7]
# start_node = (10,10)
# end_node = (90,90)
num = len(obstacle)
fig = plt.figure

plt.scatter(start_node[0], start_node[1], color='red', marker='o')
plt.scatter(end_node[0], end_node[1], color='red', marker='o')

graph = [[], []] # 正常的圖 list, 0 是 起點 1 是終點
obstacle_node = [[] for _ in range(num)] # 同一個障礙物上面的點會存在一起
point_map = {start_node: 0, end_node: 1} # 每個點對應的數字
point_index = 2 # 總共的點

#畫圓
for i in range(0,num):
    circle = plt.Circle(obstacle[i], radius[i], fill = False)
    plt.gca().add_patch(circle)
#找點切線
for i in range(0,num):
    dis1 = (start_node[0] - obstacle[i][0])**2 + (start_node[1] - obstacle[i][1])**2
    dis2 = (end_node[0] - obstacle[i][0])**2 + (end_node[1] - obstacle[i][1])**2
    theta_start = math.atan((obstacle[i][1] - start_node[1])/(obstacle[i][0] - start_node[0]))
    theta_end = math.atan((end_node[1] - obstacle[i][1])/(end_node[0] - obstacle[i][0]))
    theta1 = math.asin(radius[i]/dis1)
    theta2 = math.asin(radius[i]/dis2)
    point1 = (obstacle[i][0] + radius[i]*math.cos(theta_start + theta1 + math.pi/2), obstacle[i][1] + radius[i]*math.sin(theta_start + theta1 + math.pi/2))
    point2 = (obstacle[i][0] + radius[i]*math.cos(theta_start - theta1 - math.pi/2), obstacle[i][1] + radius[i]*math.sin(theta_start - theta1 - math.pi/2))
    point3= (obstacle[i][0] + radius[i]*math.cos(theta_end + theta2 + math.pi/2), obstacle[i][1] + radius[i]*math.sin(theta_end + theta2 + math.pi/2))
    point4= (obstacle[i][0] + radius[i]*math.cos(theta_end - theta2 - math.pi/2), obstacle[i][1] + radius[i]*math.sin(theta_end - theta2 - math.pi/2))

    flag = 0
    for j in range(0,num):
        if intersect(start_node, point1, obstacle[j], radius[j]) and j != i: # 如果有障礙物跟這條線撞到
            break
    if not flag:
        plt.plot([point1[0], start_node[0]], [point1[1], start_node[1]], linestyle='-', color='green')
        path_len = calculate_distance(start_node, point1)

        point_map[point1] = point_index # 設個對應的數字
        point_index = point_index + 1
        graph.append([]) # graph 要為新的數字加一個 list
        graph[0].append((point_index-1, path_len)) # 起點加上現在這個點
        graph[point_index-1].append((0, path_len)) # 這個點加上起點
        obstacle_node[i].append(point_index-1) # 第 i 個 obstacle 要加上現在這個點

    flag = 0
    for j in range(0,num):
        if intersect(start_node, point2, obstacle[j], radius[j]) and j != i: # 如果有障礙物跟這條線撞到
            flag = 1
            break
    if not flag:
        plt.plot([point2[0], start_node[0]], [point2[1], start_node[1]], linestyle='-', color='green')
        path_len = calculate_distance(start_node, point2)

        point_map[point2] = point_index
        point_index = point_index + 1
        graph.append([])
        graph[0].append((point_index-1, path_len)) # 起點加上現在這個點
        graph[point_index-1].append((0, path_len)) # 這個點加上起點
        obstacle_node[i].append(point_index-1)

    flag = 0
    for j in range(0,num):
        if intersect(end_node, point3, obstacle[j], radius[j]) and j != i: # 如果有障礙物跟這條線撞到
            flag = 1
            break
    if not flag:
        plt.plot([point3[0], end_node[0]], [point3[1], end_node[1]], linestyle='-', color='green')
        path_len = calculate_distance(end_node, point3)

        point_map[point3] = point_index
        point_index = point_index + 1
        graph.append([])
        graph[1].append((point_index-1, path_len)) # 起點加上現在這個點
        graph[point_index-1].append((1, path_len)) # 這個點加上起點
        obstacle_node[i].append(point_index-1)

    flag = 0
    for j in range(0,num):
        if intersect(end_node, point4, obstacle[j], radius[j]) and j != i: # 如果有障礙物跟這條線撞到
            flag = 1
            break
    if not flag:
        plt.plot([point4[0], end_node[0]], [point4[1], end_node[1]], linestyle='-', color='green')
        path_len = calculate_distance(end_node, point3)

        point_map[point4] = point_index
        point_index = point_index + 1
        graph.append([])
        graph[1].append((point_index-1, path_len)) # 起點加上現在這個點
        graph[point_index-1].append((1, path_len)) # 這個點加上起點
        obstacle_node[i].append(point_index-1)

    print(i,math.tan(theta_start))

#找2圓切線
for i in range(0,num):
    for j in range(i + 1,num):
        swap = 0
        #i的半徑比j小
        if radius[i] > radius[j]:
            swap = 1
            i, j = j, i 
        dis = math.sqrt((obstacle[i][0] - obstacle[j][0])**2 + (obstacle[i][1] - obstacle[j][1])**2)
        rad_diff = radius[j] - radius[i]    #2圓半徑差
        rad_add = radius[i] + radius[j]     #2圓半徑和
        theta0 = math.asin(rad_diff/dis)    #2圓間的角度
        theta1 = math.asin(rad_add/dis)
        theta2 = math.atan((obstacle[j][1] - obstacle[i][1])/(obstacle[j][0] - obstacle[i][0])) #與水平線夾角
        #外公切線
        theta3 = theta0 + theta2 + math.pi/2
        theta4 = theta2 - theta0 - math.pi/2
        if(obstacle[i][0] - obstacle[j][0] > 0):
                theta3 = theta3 + math.pi
                theta4 = theta4 + math.pi
        point1 = (obstacle[i][0] + radius[i] * math.cos(theta3), obstacle[i][1] + radius[i] * math.sin(theta3))
        point2 = (obstacle[j][0] + radius[j] * math.cos(theta3), obstacle[j][1] + radius[j] * math.sin(theta3))
        point3 = (obstacle[i][0] + radius[i] * math.cos(theta4), obstacle[i][1] + radius[i] * math.sin(theta4))
        point4 = (obstacle[j][0] + radius[j] * math.cos(theta4), obstacle[j][1] + radius[j] * math.sin(theta4))
        for k in range(0,num):
            v0 = (point1[0] - point2[0], point1[1] - point2[1])
            D = (point1[0]*v0[0] + point1[1]*v0[1] - v0[0]*obstacle[k][0] - v0[1]*obstacle[k][1])**2 - (v0[0]**2 + v0[1]**2 )*(point1[0]**2 + point1[0]**2 + obstacle[k][0]**2 + obstacle[k][1]**2 - radius[k]**2 - 2*point1[0]*obstacle[k][0] - 2*point1[1]*obstacle[k][1])
            
            #print(i,j,k,D)

        # plt.plot([point1[0], point2[0]], [point1[1], point2[1]], linestyle='-', color='b')
        # plt.plot([point3[0], point4[0]], [point3[1], point4[1]], linestyle='-', color='b')

        #內公切線
        if(dis == radius[i]  + radius[j]):
            point0 = (obstacle[i][0] + radius[i] * math.cos(theta2), obstacle[i][1] + radius[i] * math.sin(theta2))
            plt.scatter(point0[0], point0[1], color='red', marker='o')
        elif(dis > radius[i]  + radius[j]):
            theta5 = theta2 - theta1 + math.pi/2
            theta6 = theta2 + theta1 - math.pi/2
            if(obstacle[i][0] - obstacle[j][0] > 0):
                theta5 = theta5 + math.pi
                theta6 = theta6 + math.pi
            point5 = (obstacle[i][0] + radius[i] * math.cos(theta5), obstacle[i][1] + radius[i] * math.sin(theta5))
            point6 = (obstacle[j][0] + radius[j] * math.cos(theta5-math.pi), obstacle[j][1] + radius[j] * math.sin(theta5-math.pi))
            point7 = (obstacle[i][0] + radius[i] * math.cos(theta6), obstacle[i][1] + radius[i] * math.sin(theta6))
            point8 = (obstacle[j][0] + radius[j] * math.cos(theta6-math.pi), obstacle[j][1] + radius[j] * math.sin(theta6-math.pi))
            # plt.plot([point5[0], point6[0]], [point5[1], point6[1]], linestyle='-', color='b')
            # plt.plot([point7[0], point8[0]], [point7[1], point8[1]], linestyle='-', color='b')
        else:
            print('not cut line')
 
        
        #print(i,j,dis,math.degrees(theta0))
        if swap == 1:
            i, j = j, i 

plt.xlim(0,100)
plt.ylim(0,100)                        
plt.show()