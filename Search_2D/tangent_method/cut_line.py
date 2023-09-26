import matplotlib.pyplot as plt
import numpy as np
import math

obstacle = [(44,44),(55,55),(66,66)]#[(56,30),(62,38),(74,28),(37,99),(55,11),(67,12),(43,44),(78,71),(22,20)]
radius = [5,7,5] #[5,5,4,7,6,3,8,2,7]
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
#找切線
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
            
            print(i,j,k,D)

        plt.plot([point1[0], point2[0]], [point1[1], point2[1]], linestyle='-', color='b')
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
            plt.plot([point5[0], point6[0]], [point5[1], point6[1]], linestyle='-', color='b')
            plt.plot([point7[0], point8[0]], [point7[1], point8[1]], linestyle='-', color='b')
        else:
            print('not cut line')
 
        
        #print(i,j,dis,math.degrees(theta0))
        if swap == 1:
            i, j = j, i 

plt.xlim(0,100)
plt.ylim(0,100)                        
plt.show()