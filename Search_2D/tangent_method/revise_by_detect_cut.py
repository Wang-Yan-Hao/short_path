import matplotlib.pyplot as plt
import numpy as np
import math

def check_cut(p1,p2,o,r):
    v = (p2[0]-p1[0], p2[1]-p1[1])
    #(p1[0] + t*v[0] - obstacle[0][0])**2 + (p1[1] + t*v[1] - obstacle[0][1])**2 = radius[0]**2
    #(v[0]**2 + v[1]**2)*t^2 + 2*(p1[0]*v[0] - v[0]*obstacle[0][0] + p1[1]*v[1] - v[1]*obstacle[0][1])*t + 
    #(p1[0]**2 + obstacle[0][0]**2 + p1[1]**2 + obstacle[0][1]**2 - radius[0]**2 - 2*p1[0]*obstacle[0][0] - 2*p1[1]*obstacle[0][1]) = 0
    a = v[0]**2 + v[1]**2
    b = 2*(p1[0]*v[0] - v[0]*o[0] + p1[1]*v[1] - v[1]*o[1])
    c = p1[0]**2 + o[0]**2 + p1[1]**2 + o[1]**2 - r**2 - 2*p1[0]*o[0] - 2*p1[1]*o[1]
    D = b**2 - 4*a*c
    if D > 0:
        t1 = (-b + math.sqrt(D))/(2*a)
        t2 = (-b - math.sqrt(D))/(2*a)
        #print(t1,t2)
        K = (p1[0] + t1*v[0],p1[1] + t1*v[1])
        K1 = (p1[0] + t2*v[0],p1[1] + t2*v[1])
        #plt.scatter(K[0], K[1], color='blue', marker='o')
        #plt.scatter(K1[0], K1[1], color='blue', marker='o')
        if (t1 >= 0) & (t1 <=1) & (t2 >= 0) & (t2 <=1):
            return 1
    #print(a,b,c,D)
    #plt.show()
    return 0

def find_point_cut(point,o,r):
    flag1 = 0
    flag2 = 0
    dis = math.sqrt((point[0] - o[0])**2 + (point[1] - o[1])**2)
    theta= math.atan((point[1] - o[1])/(point[0] - o[0]))
    theta1 = math.asin(r/dis)
    point1 = (o[0] + r*math.cos(theta + theta1 + math.pi/2), o[1] + r*math.sin(theta + theta1 + math.pi/2))
    point2 = (o[0] + r*math.cos(theta - theta1 - math.pi/2), o[1] + r*math.sin(theta - theta1 - math.pi/2))
    for i in range(0,num):
        ans = check_cut(point, point1, obstacle[i], radius[i])
        if ans == 1:
            flag1 = 1
            break
    for i in range(0,num):
        ans = check_cut(point, point2, obstacle[i], radius[i])
        if ans == 1:
            flag2 = 1
            break
    if flag1 == 0:
        plt.plot([point1[0], point[0]], [point1[1], point[1]], linestyle='-', color='green')
    if flag2 == 0:
        plt.plot([point2[0], point[0]], [point2[1], point[1]], linestyle='-', color='green')

    
obstacle = [(56,30),(62,38),(74,28),(37,99),(55,11),(67,12),(43,44),(78,71),(22,20)]
radius = [5,5,4,7,6,3,8,2,7]
start_node = (10,10)
end_node = (90,90)
num = len(obstacle)
fig = plt.figure

plt.scatter(start_node[0], start_node[1], color='red', marker='o')
plt.scatter(end_node[0], end_node[1], color='red', marker='o')
#畫圓
for i in range(0,num):
    circle = plt.Circle(obstacle[i], radius[i], fill = False)
    plt.gca().add_patch(circle)
#找點切線
for i in range(0,num):
    find_point_cut(start_node, obstacle[i], radius[i])
    find_point_cut(end_node, obstacle[i], radius[i])

#找2圓切線
for i in range(0,num):
    for j in range(i + 1,num):
        swap = 0
        #i的半徑比j小
        if radius[i] > radius[j]:
            swap = 1
            i, j = j, i 
        flag1 = 0
        flag2 = 0
        flag3 = 0
        flag4 = 0
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
            ans = check_cut(point1, point2, obstacle[k], radius[k])
            if ans == 1:
                flag1 = 1
                break
        for k in range(0,num):
            ans = check_cut(point3, point4, obstacle[k], radius[k])
            if ans == 1:
                flag2 = 1
                break
        if flag1 == 0:
            plt.plot([point1[0], point2[0]], [point1[1], point2[1]], linestyle='-', color='b')
        if flag2 == 0:
            plt.plot([point3[0], point4[0]], [point3[1], point4[1]], linestyle='-', color='b')
        
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
            for k in range(0,num):
                ans = check_cut(point5, point6, obstacle[k], radius[k])
                if ans == 1:
                    flag3 = 1
                    break
            for k in range(0,num):
                ans = check_cut(point7, point8, obstacle[k], radius[k])
                if ans == 1:
                    flag4 = 1
                    break
            if flag3 == 0:
                plt.plot([point5[0], point6[0]], [point5[1], point6[1]], linestyle='-', color='b')
            if flag4 == 0:
                plt.plot([point7[0], point8[0]], [point7[1], point8[1]], linestyle='-', color='b')
        else:
            print('not cut line')
 
        
        #print(i,j,dis,math.degrees(theta0))
        if swap == 1:
            i, j = j, i 

plt.xlim(0,100)
plt.ylim(0,100)                        
plt.show()