import heapq
import numpy as np

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

    return path, distances[goal]