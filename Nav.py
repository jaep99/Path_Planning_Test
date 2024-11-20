import matplotlib.pyplot as plt
import time, serial, threading, sys
from Djikstra import parse_geojson, shortest_xy_path, xy_to_gps, gps_to_xy
from ld06 import take_measurements
import ld06
import PotentialField as pf


# curr ALWAYS NEEDS TO BE LIST SO THAT THREAD ARGUEMENT IS UPDATED
curr = [(0,0)]
def get_curr(vehicle):
    '''
    curr_gps = vehicle.location.global_relative_frame
    return gps_to_xy(curr_gps)
    '''


adjacency = parse_geojson("CampusMap.geojson")

# manually found culc and student center coordinates
culc = (-84.3965628, 33.7750484)
stuce = (-84.3981533, 33.7739460)

# Find the shortest path from CULC to Student Center 
shortest_path = shortest_xy_path(adjacency, culc, stuce)

shortest_path = [(0,0), (0,6)]
# wp ALWAYS NEEDS TO BE LIST SO THAT THREAD ARGUEMENT IS UPDATED
wp = [shortest_path[0]]


#start thread to update obstacles
lidar_serial = serial.Serial("/dev/ttyUSB0", baudrate=230400, timeout= .5)
lidarThread = threading.Thread(target = take_measurements, args = (lidar_serial, curr, wp), daemon= True)
lidarThread.start()
time.sleep(10)


# TODO send first waypoint to pixhawk here
# will have to convert to gps coord when sending using xy_to_gps from Djikstra.py 
path = [xy_to_gps(shortest_path[0], reference = shortest_path[0])] # keep track of path to plot

# TODO start a thread that updates curr with current location 
# locationThread = threading.Thread(target = update_curr, args = (vehicle,), daemon= True)
# locationThread.start()


try:
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    plt.show()
    i = 1
    while(curr != stuce):
        while(curr != shortest_path[i]):
            #go to waypoint
            print("got here")
            wp = [pf.potential_field(curr, wp, shortest_path[i])] #updates wp according to obstacles and goal
            curr = wp #temporary for simulation
            path.append(wp[0])
            print(path)

            obstacles_x = [obstacle[0] for obstacle in ld06.obstacles]
            obstacles_y = [obstacle[1] for obstacle in ld06.obstacles]
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            desired_xs = [point[0] for point in shortest_path]
            desired_ys = [point[1] for point in shortest_path]

            ax.clear()  # Clear the axes instead of recreating the plot
            ax.scatter(obstacles_x, obstacles_y, label = "obstacles")
            ax.plot(path_x, path_y, label = "path")
            ax.scatter(desired_xs, desired_ys, label = "desired waypoint")
            ax.set_xlim(-7, 7)
            ax.set_ylim(-7, 7)
            plt.pause(2)  # Pause for 2 seconds instead of using `time.sleep`

        print("got to wp")
        i+=1
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting")
finally:
    plt.close()
    plt.ioff()
    lidarThread.join()
    sys.exit()

