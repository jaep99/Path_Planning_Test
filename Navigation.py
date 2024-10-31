import json 
import numpy as np
import matplotlib.pyplot as plt
from djikstra import dijkstra
import random

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

#plot map to check adjacency
# for start in adjacency.keys():
#     for end in adjacency[start]:
#         plt.plot([start[0],end[0]], [start[1],end[1]])
# plt.show()

# manually found culc and student center coordinates
culc = (-84.3965628, 33.7750484)
stuce = (-84.3981533, 33.7739460)

# Find the shortest path from CULC to Student Center
shortest_path, total_distance = dijkstra(adjacency, culc, stuce)

#simulate following path with a hidden obstacle
#obstacles = [(-84.3968652, 33.7752030), (-84.3973144, 33.7752068)]
current = shortest_path[0]
print(shortest_path)

paths = [[current]]
obstacles = []

i = 1
while(i < len(shortest_path)):
    next_wp = shortest_path[i]

    # recalculate whole path method
    #if not (next_wp in obstacles):
    # This is where we would send mavproxy a waypoint
    # else:
    #     adjacency[current].remove(next_wp)
    #     shortest_path, _ = dijkstra(adjacency, current, stuce)
    #     i = 0
    #     paths.append([current])

    # add waypoint to avoid obstacle
    if(random.random() < .5):
        direction = np.subtract(next_wp, current)
        midpoint = (current[0] + direction[0]/2, current[1] + direction[1]/2)

        perpindicular = np.array([-direction[1], direction[0]])
        perpindicular = perpindicular/np.linalg.norm(perpindicular)

        obstacles.append(midpoint)
        
        next_wp = (midpoint[0] + .00005*perpindicular[0], midpoint[1]+ .00005*perpindicular[1])
        i -= 1

    
    current = next_wp
    paths[-1].append(current) 
    i += 1

for start in adjacency.keys():
    for end in adjacency[start]:
        plt.plot([start[0], end[0]], [start[1], end[1]], color='gray')

pathnum = 1
for path in paths:
    color = (random.random(), random.random(), random.random())
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        plt.plot([start[0], end[0]], [start[1], end[1]], color=color, label=f'Path_{pathnum}' if i == 0 else "")
    pathnum += 1
for obstacle in obstacles:
    plt.scatter(obstacle[0], obstacle[1], color = 'red', label='Obstacles' if obstacle == obstacles[0] else "")
plt.legend()
plt.show()