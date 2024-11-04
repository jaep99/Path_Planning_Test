import json 
import numpy as np
import matplotlib.pyplot as plt
from Djikstra import dijkstra
import random
from math import sqrt, cos, radians


with open('CampusMap.geojson') as f:
    campus_json = json.load(f)

paths = campus_json['features']
adjacency = {} # key: start coordinate, value: list of end coordinates

# create dict storing all edges
for path in paths:
    coordinates = path['geometry']['coordinates']
    for i in range(len(coordinates)-1):
        point1 = tuple(coordinates[i])
        point2 = tuple(coordinates[i+1])

        if point1 in adjacency.keys():
            adjacency[point1].append(point2)
        else:
            adjacency[point1] = [point2]

        if point2 in adjacency.keys():
            adjacency[point2].append(point1)
        else:
            adjacency[point2] = [point1]

# manually found culc and student center coordinates
culc = (-84.3965628, 33.7750484)
stuce = (-84.3981533, 33.7739460)

# Find the shortest path from CULC to Student Center
shortest_path, total_distance = dijkstra(adjacency, culc, stuce)

paths = [shortest_path[0]]
obstacles = []
detected = []

global wp
global prev_wp
wp = shortest_path[0]

i = 1
done = False
while(not done): 
    prev_wp = wp
    wp = shortest_path[i]

    paths.append(wp)  #send wp to maxproxy

    #wait for actual drone location to equal waypoint here
    #i.e. while(current_location != wp)

    if(random.random() < .5): #simulates interrupt from LIDAR 
        #all of the followning code would be in an interrupt
        #goal of interrupt is to alter wp

        #read current location, currently randomly placed between prev and next wp 
        current_distance = random.random()
        current_location = tuple(np.add(current_distance*np.subtract(wp, prev_wp), prev_wp))

        #distance in meters from actual drone location to wp
        R = 6371000  # radius of the earth in meters
        x = (radians(wp[0]) - radians(current_location[0])) * cos(0.5 * (radians(wp[1]) + radians(current_location[1])))
        y = radians(wp[1]) - radians(current_location[1])
        d_to_wp = R * sqrt(x*x + y*y)

        #LIDAR measurement, currently randomly placed and assuming LIDAR threshold of 5m
        if d_to_wp > 5:
            obstacle_distance = 5
        else:
            obstacle_distance = random.random()*d_to_wp + .1

        direction = np.subtract(wp, current_location)*obstacle_distance/d_to_wp
        
        obstacle = (current_location[0] + direction[0], current_location[1] + direction[1]) 

        obstacles.append(obstacle)
        detected.append(current_location)

        perpindicular = np.array([-direction[1], direction[0]])
        perpindicular = perpindicular/np.linalg.norm(perpindicular)

        prev_wp = wp

        #check both directions here by rotating drone 
        # in mavproxy set mode to circle with radius 0
        if random.random() < .5: #simulate first direction free
            wp = (obstacle[0] + .00005*perpindicular[0], obstacle[1] + .00005*perpindicular[1]) 
            index = paths.index(prev_wp) #this won't matter with maxproxy
            paths.insert(index, wp) #in maxproxy clear old way point and set this as wp
            paths.insert(index, current_location) #this won't matter with maxproxy
        else: #simulate second direction free
            wp = (obstacle[0] - .00005*perpindicular[0], obstacle[1] - .00005*perpindicular[1]) 
            index = paths.index(prev_wp) #this won't matter with maxproxy
            paths.insert(index, wp) #in maxproxy clear old way point and set this as wp
            paths.insert(index, current_location) #this won't matter with maxproxy
        # else: #neither free 
        #     #return to prev_wp 
        #     #recaclulate path from there
        #     adjacency[prev_wp].remove(wp)
        #     shortest_path, _ = dijkstra(adjacency, prev_wp, stuce)
        #     i = 0
        #     paths.append()

        i -= 1 # so that wp in next loop is the original target wp

    #break if 
    if wp == stuce:
        break
    
    i += 1

for start in adjacency.keys():
    for end in adjacency[start]:
        plt.plot([start[0], end[0]], [start[1], end[1]], color='gray')

print(path)
for i in range(len(paths) - 1):
    start = paths[i]
    end = paths[i + 1]
    if i%2 == 0:
        color = 'blue'
    else:
        color = 'green'
    plt.plot([start[0], end[0]], [start[1], end[1]], color = color)

for obstacle in obstacles:
    plt.scatter(obstacle[0], obstacle[1], color = 'red', label='Obstacles' if obstacle == obstacles[0] else "")
for point in detected:
    plt.scatter(point[0], point[1], color = 'black', label='Detected' if point == detected[0] else "")
plt.legend()
plt.show()