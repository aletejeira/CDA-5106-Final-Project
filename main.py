import carla
import time
import xml.etree.ElementTree as ET
import math
import random
from PCLA import PCLA
from fault_injector import FaultInjector
from sensor_fault import CameraFault, Occlusion, RectangleMask, Gaussian, Noise, Blackout, SpeedometerFault, SpeedometerBias, GNSSFault, GNSSDrift

def get_route_destination(route_file):
    """Extract the final waypoint from the route XML"""
    tree = ET.parse(route_file)
    root = tree.getroot()
    waypoints = root.findall('waypoint')
    if not waypoints:
        return None
    last_waypoint = waypoints[-1]
    x = float(last_waypoint.get('x'))
    y = float(last_waypoint.get('y'))
    z = float(last_waypoint.get('z'))
    return carla.Location(x, y, z)

def is_destination_reached(current_location, destination_location, threshold=3.0):
    """Check if the current location is within a certain distance of the destination."""
    dx = current_location.x - destination_location.x
    dy = current_location.y - destination_location.y
    dz = current_location.z - destination_location.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    return distance <= threshold

# Spawn traffic and make them move around via the Traffic Manager
def spawn_traffic(world, traffic_manager, num_vehicles=50):
    """Spawn traffic vehicles and enable autopilot so they actually move."""
    bp_library = world.get_blueprint_library()
    vehicle_bps = bp_library.filter('vehicle.*')

    # Filter out large vehicles (CARLA 0.9.16)
    exclude = (
    "bus",          # catches ids that literally contain "bus"
    "fusorosa",     # vehicle.mitsubishi.fusorosa (coach bus)
    "t2",           # vehicle.volkswagen.t2 (microbus/van-like)
    "truck",
    "european_hgv", 
    "carlacola",
    "firetruck",
    "ambulance",
    "van",
    "crossbike", # exludes bicycles
    "century",
    "omafiets",
    )
    vehicle_bps = [bp for bp in vehicle_bps if not any(k in bp.id.lower() for k in exclude)]

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    tm_port = traffic_manager.get_port() if hasattr(traffic_manager, "get_port") else None

    spawned = 0
    for spawn_point in spawn_points:
        if spawned >= num_vehicles:
            break

        bp = random.choice(vehicle_bps)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "autopilot")
        if bp.has_attribute("color"):
            colors = bp.get_attribute("color").recommended_values
            if colors:
                bp.set_attribute("color", random.choice(colors))
        if bp.has_attribute("driver_id"):
            driver_ids = bp.get_attribute("driver_id").recommended_values
            if driver_ids:
                bp.set_attribute("driver_id", random.choice(driver_ids))

        vehicle = world.try_spawn_actor(bp, spawn_point)
        if vehicle is None:
            continue

        # Hand over control to the Traffic Manager
        if tm_port is None:
            vehicle.set_autopilot(True)
        else:
            vehicle.set_autopilot(True, tm_port)

        # Slight speed randomness so traffic doesn't bunch up
        if hasattr(traffic_manager, "vehicle_percentage_speed_difference"):
            traffic_manager.vehicle_percentage_speed_difference(vehicle, random.randint(-20, 20))

        # Make cars slower to better observe interactions; adjust as needed for denser traffic or faster flow
        # traffic_manager.vehicle_percentage_speed_difference(vehicle, 55)

        # Encourage vehicles to "move around" (lane changes) when supported by this CARLA version
        if hasattr(traffic_manager, "auto_lane_change"):
            traffic_manager.auto_lane_change(vehicle, True)
        if hasattr(traffic_manager, "random_left_lanechange_percentage"):
            traffic_manager.random_left_lanechange_percentage(vehicle, random.randint(0, 30))
        if hasattr(traffic_manager, "random_right_lanechange_percentage"):
            traffic_manager.random_right_lanechange_percentage(vehicle, random.randint(0, 30))

        spawned += 1

def main():

    # HOST_IP  = "localhost"
    HOST_IP  = "172.17.112.1" # Connect to windows CALRA server from WSL2
    client = carla.Client(HOST_IP, 2000)
    # client.set_timeout(10.0)
    client.set_timeout(100.0)
    client.load_world("Town02")
    synchronous_master = False
    pcla = None
    settings = None

    # Initialize fault injector and faults
    fault_injector = FaultInjector(client)
    rectangle_mask_fault = RectangleMask()
    gaussian_noise_fault = Gaussian()
    blackout_fault = Blackout()
    speedometer_bias_fault = SpeedometerBias() 
    gnss_drift_fault = GNSSDrift()

    try:
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8050)
        
        settings = world.get_settings()
        # NEw
        print("sync:", settings.synchronous_mode, "fixed_dt:", settings.fixed_delta_seconds)
        #####
        asynch = False
        if not asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                # settings.fixed_delta_seconds = 0.10  # 2x faster sim time per tick
                # settings.fixed_delta_seconds = 0.20  # 4x faster

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

        # Spawn vehicle
        vehicle = world.spawn_actor(vehicleBP, vehicle_spawn_points[31])
        
        # Retrieve the spectator object
        spectator = world.get_spectator()

        # How far behind / above the car (in meters)
        follow_distance = 8.0
        follow_height   = 3.0
        pitch_down_deg  = -15.0  # look slightly down

        def follow_ego_spectator(vehicle):
            v_tf = vehicle.get_transform()

            # Offset in vehicle LOCAL coordinates: (-x is behind, +z is up)
            offset_local = carla.Location(x=-follow_distance, z=follow_height)

            # Convert local offset to WORLD location
            cam_location = v_tf.transform(offset_local)

            # Aim spectator with the vehicle yaw (optionally keep roll=0)
            cam_rotation = carla.Rotation(
                pitch=pitch_down_deg,
                yaw=v_tf.rotation.yaw,
                roll=0.0
            )

            spectator.set_transform(carla.Transform(cam_location, cam_rotation))

        # Set the spectator with our transform
        # spectator.set_transform(carla.Transform(carla.Location(x=-8, y=108, z=7), carla.Rotation(pitch=-19, yaw=0, roll=0)))

        # Attach spectator behind the vehicle (in the vehicle's local frame)
       

        # Spawn traffic
        spawn_traffic(world, traffic_manager, num_vehicles=50)


        world.tick()

        # Using World on Rails (WoR) agent (No Crash Version)
        agent =  "wor_nc"
        # agent = "lbc_nc" # very slow driving speed

        # Leaderboard version 
        # agent = "wor_lb"
        # agent = "lbc_lb" # very slow

        # route = "./sample_route.xml"
        route = "experiment_route.xml"

        destination_location = get_route_destination(route)
        if destination_location is not None:
            print(f"Destination set to: ({destination_location.x:.2f}, {destination_location.y:.2f}, {destination_location.z:.2f})")
        else:
            print("Error: Could not parse route destination.")

        # Create pcla framework instance
        pcla = PCLA(agent, vehicle, route, client)

        print("Ego speed limit (km/h):", vehicle.get_speed_limit())
        
      


        # Attach the fault directly to the agent; ImageAgent.run_step will apply it.
        # pcla.agent_instance.rectangle_mask_fault = rectangle_mask_fault
        pcla.agent_instance.gaussian_noise_fault = gaussian_noise_fault
        # pcla.agent_instance.speed_bias_fault = speedometer_bias_fault
        # pcla.agent_instance.gnss_drift_fault = gnss_drift_fault
        pcla.agent_instance.blackout_fault = blackout_fault

        step = 0
        while True:
            try:

                if is_destination_reached(pcla.vehicle.get_location(), destination_location): 
                    print("Destination reached!")
                    break
                else:
                    if step % 20 == 0:  # Print status every 20 steps
                        current_location = pcla.vehicle.get_location()
                        distance_to_dest = current_location.distance(destination_location)
                        print(f"Step {step}: Distance to destination: {distance_to_dest:.2f} meters")
                follow_ego_spectator(vehicle)   # or pcla.vehicle
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
        # evaluator.print_report()

    
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
