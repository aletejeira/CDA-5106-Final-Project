import carla
import time
import xml.etree.ElementTree as ET
import math
from PCLA import PCLA



def get_route_start(route_file):
    """Extract the first waypoint from the route XML"""
    tree = ET.parse(route_file)
    root = tree.getroot()
    first_waypoint = root.find('waypoint')
    if first_waypoint is None:
        return None
    x = float(first_waypoint.get('x'))
    y = float(first_waypoint.get('y'))
    z = float(first_waypoint.get('z'))
    return carla.Location(x, y, z)

def distance(loc1, loc2):
    """Calculate distance between two locations"""
    dx = loc1.x - loc2.x
    dy = loc1.y - loc2.y
    dz = loc1.z - loc2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)


def main():

    # HOST_IP  = "localhost"
    HOST_IP  = "172.17.112.1" # Connect to windows CALRA server from WSL2
    client = carla.Client(HOST_IP, 2000)
    # client.set_timeout(10.0)
    client.set_timeout(100.0)
    client.load_world("Town02")
    # client.load_world("Town01")
    synchronous_master = False
    pcla = None
    settings = None

    try:
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)
        # traffic_manager = client.get_trafficmanager(8150)
        
        settings = world.get_settings()
        asynch = False
        if not asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
                    you could experience some issues. If it's not working correctly, switch to \
                    synchronous mode by using traffic_manager.set_synchronous_mode(True)")
        #settings.no_rendering_mode = True
        world.apply_settings(settings)
        
        # Finding actors
        bpLibrary = world.get_blueprint_library()

        ## Finding vehicle
        vehicleBP = bpLibrary.filter('model3')[0]

        vehicle_spawn_points = world.get_map().get_spawn_points()

        ### Spawn vehicle
        vehicle = world.spawn_actor(vehicleBP, vehicle_spawn_points[31])
        # vehicle = world.spawn_actor(vehicleBP, vehicle_spawn_points[29]) # Too far from route start
        # vehicle = world.spawn_actor(vehicleBP, vehicle_spawn_points[4]) # Too far from route start
        
        # Retrieve the spectator object
        spectator = world.get_spectator()

        # Set the spectator with our transform
        # spectator.set_transform(carla.Transform(carla.Location(x=-8, y=108, z=7), carla.Rotation(pitch=-19, yaw=0, roll=0)))

        # Attach spectator behind the vehicle (in the vehicle's local frame)
        vehicle_tf = vehicle.get_transform()
        spectator_location = vehicle_tf.transform(carla.Location(x=-8, y=0, z=7))
        spectator_rotation = carla.Rotation(
            pitch=-19,
            yaw=vehicle_tf.rotation.yaw,
            roll=vehicle_tf.rotation.roll,
        )
        spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))

        world.tick()

        # agent = "tfv5_alltowns"
        # agent = "simlingo_simlingo" # Too slow, often causes timeouts
        # agent = "lmdrive_llava"
        # agent = "if_if"
        # agent = "carl_carl_0" # Sorta okay but still broken
        # agent = "neat_neat"
        # agent = "tfv6_regnet" #didnt work
        # agent = "tfv4_l6_0"
        # agent = "lav_lav"
        # agent =  "wor_nc"
        agent = "wor_lb"
        # 
        route = "./sample_route.xml"
        # route = "./new_route.xml"
        pcla = PCLA(agent, vehicle, route, client)
        
        # Check if vehicle is near route start
        route_start = get_route_start(route)
        vehicle_pos = vehicle.get_location()
        dist = distance(vehicle_pos, route_start)
        print('\nSpawned the vehicle with model =', agent,', press Ctrl+C to exit.\n')
        print(f'Vehicle position: ({vehicle_pos.x:.1f}, {vehicle_pos.y:.1f}, {vehicle_pos.z:.1f})')
        print(f'Route start: ({route_start.x:.1f}, {route_start.y:.1f}, {route_start.z:.1f})')
        print(f'Distance to route start: {dist:.1f} meters')
        if dist > 10:
            print('⚠️  WARNING: Vehicle is far from route start! Agent may struggle to navigate.\n')
        else:
            print('✓ Vehicle is close to route start.\n')
        
        step = 0
        while True:
            try:
                ego_action = pcla.get_action()
                vehicle.apply_control(ego_action)
                world.tick()
                step += 1
            except Exception as e:
                print(f'\nError at step {step}:')
                print(f'{type(e).__name__}: {e}\n')
                import traceback
                traceback.print_exc()
                break
    
    finally:
        #settings.no_rendering_mode = False
        if settings is not None:
            settings.synchronous_mode = False
            world.apply_settings(settings)

        # Destroy vehicle
        print('\nCleaning up the vehicle')
        if pcla is not None:
            pcla.cleanup()
        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('Done.')
