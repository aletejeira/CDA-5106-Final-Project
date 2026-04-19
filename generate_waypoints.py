from PCLA import location_to_waypoint
import carla
from PCLA import route_maker

HOST_IP = "172.17.112.1" # Connect to windows CALRA server from WSL2
client = carla.Client(HOST_IP, 2000)
# client.set_timeout(10.0)
client.set_timeout(20.0)
client.load_world("Town02")


world = client.get_world()
vehicle_spawn_points = world.get_map().get_spawn_points() # Get CARLA spawn points
startLoc = vehicle_spawn_points[31].location              # Define Start location
endLoc = vehicle_spawn_points[4].location                # Define End location
# startLoc = vehicle_spawn_points[29].location              # Define Start location
# endLoc = vehicle_spawn_points[22].location                # Define End location

# Returns a list of CARLA waypoints between the two locations
waypoints = location_to_waypoint(client, startLoc, endLoc) 

route_maker(waypoints, "experiment_route.xml")
