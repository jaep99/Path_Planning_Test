import heapq
import numpy as np
import json
from math import cos, radians,degrees


def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Dijkstra's algorithm implementation
def dijkstra(adjacency, start, goal):
    # Priority queue and distance dictionary
    queue = [(0, start)]
    distances = {point: float('inf') for point in adjacency.keys()}
    distances[start] = 0
    previous_nodes = {point: None for point in adjacency.keys()}
    
    while queue:
        current_distance, current_point = heapq.heappop(queue)

        # If we reached the goal, stop
        if current_point == goal:
            break

        # Check all neighbors of the current point
        for neighbor in adjacency[current_point]:
            distance = current_distance + euclidean_distance(current_point, neighbor) 
            
            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_point
                heapq.heappush(queue, (distance, neighbor))
    
    # Reconstruct path
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()

    return path

#returns dict with all adjacencies
def parse_geojson(fileName):
    with open(fileName) as f:
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

    return adjacency

#return shortest path in xy coordinates with origin at start
def shortest_xy_path(adjacency, start, end):
    gps_path = dijkstra(adjacency, start, end)
    start = gps_path[0]
    xy_path = [gps_to_xy(gps_coord, start) for gps_coord in gps_path]
    return xy_path

#converts single gps coordinate to xy point with origin at reference
def gps_to_xy(gps_coord, reference): 
    R = 6371000  # radius of the earth in meters
    x = R*((radians(gps_coord[0]) - radians(reference[0])) * cos(0.5 * (radians(gps_coord[1]) + radians(reference[1]))))
    y = R*(radians(gps_coord[1]) - radians(reference[1]))
    return (x,y)

def xy_to_gps(xy_coord, reference):
    R = 6371000  # radius of the earth in meters
    (x, y) = xy_coord
    
    # Calculate the GPS latitude and longitude
    gps_lat = reference[0] + degrees(x / R / cos(radians(reference[1])))
    gps_lon = reference[1] + degrees(y / R)
    
    return (gps_lat, gps_lon)