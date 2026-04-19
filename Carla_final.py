# works well and stops
# pre determined path and ends
import carla
import pygame
import numpy as np
import sys
import traceback
import time
import struct
import random
import csv

HOST = "localhost"
PORT = 2000
TM_PORT = 8000
DT = 0.05
W = 1280
H = 720

reachDist = 18.0  # metres

# fault config
faultOn = True

faultIntMin = 20.0
faultIntMax = 40.0
faultDurMin = 0.3
faultDurMax = 1.0


faultMode = "compound" # single or compound

# faults available to run in compound
faultTypes = {
    "steer_left": True,
    "steer_right": False,  # too unstable in compound runs
    "brake": True,
    "throttle": False,   # re-enable if needed
    # "bit_flip": True,
}

steerAmt = 0.3
brakeAmt = 0.5
throttleAmt = 0.4


# Added by Alejandro
# def bit_flip(value):
#     # Convert float to int bits
#     bits = struct.unpack('I', struct.pack('f', value))[0]
#
#     # Flip random bit
#     bit_to_flip = random.randint(0, 31)
#     bits ^= (1 << bit_to_flip)
#
#     # Convert back
#     return struct.unpack('f', struct.pack('I', bits))[0]


def apply_single_fault(ctrl, fault_name):
    if fault_name == "steer_left":
        ctrl.steer = max(-1.0, ctrl.steer - steerAmt)
    elif fault_name == "steer_right":
        ctrl.steer = min(1.0, ctrl.steer + steerAmt)
    elif fault_name == "brake":
        ctrl.throttle = 0.0
        ctrl.brake = brakeAmt
    elif fault_name == "throttle":
        ctrl.throttle = throttleAmt
        ctrl.brake = 0.0
    # elif fault_name == "bit_flip":
    #     ctrl.steer = bit_flip(ctrl.steer)


def pick_fault():
    enabled = [k for k, v in faultTypes.items() if v]
    if not enabled:
        return [], False

    if faultMode == "compound": # single fault
        return enabled, True
    else:
        return [random.choice(enabled)], False


def drawTxt(surf, fnt, txt, pos, col=(255, 255, 255)):
    sh = fnt.render(txt, True, (0, 0, 0))
    lb = fnt.render(txt, True, col)
    surf.blit(sh, (pos[0] + 1, pos[1] + 1))
    surf.blit(lb, pos)


def showErr(screen, fnt, clk, msg):
    # predates drawTxt helper, never bothered to port it over
    print(f"\nERROR: {msg}", file=sys.stderr)
    screen.fill((40, 0, 0))
    screen.blit(fnt.render("ERROR", True, (255, 80, 80)), (20, 20))
    screen.blit(fnt.render("press any key", True, (255, 255, 255)), (20, H - 40))
    y = 60
    for ln in msg.splitlines():
        screen.blit(fnt.render(ln, True, (255, 255, 255)), (20, y))
        y += 22
    pygame.display.flip()
    wait = True
    while wait:
        for event in pygame.event.get():
            if event.type in (pygame.QUIT, pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN):
                wait = False
        clk.tick(30)


pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("CARLA Follow Cam")
font = pygame.font.SysFont("consolas", 18)
clk = pygame.time.Clock()

screen.fill((20, 20, 20))
drawTxt(screen, font, "Connecting", (W // 2 - 120, H // 2 - 10))
pygame.display.flip()

car = None
cam = None
world = None
client = None

try:
    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)

    drawTxt(screen, font, "Loading map", (W // 2 - 100, H // 2 + 20))
    pygame.display.flip()

    world = client.load_world("Town10HD")
    cmap = world.get_map()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)

    allSpawns = cmap.get_spawn_points()

    tLights = list(world.get_actors().filter("traffic.traffic_light"))
    if len(tLights) < 4:
        raise RuntimeError("Not enough traffic lights.")

    # greedy "spread 4 lights apart" - picks farthest from already picked
    def pick_spread(lights, n=4):
        picked = [lights[0]]
        left = lights[1:]
        while len(picked) < n and left:
            best = max(left, key=lambda tl: min(
                tl.get_location().distance(c.get_location()) for c in picked))
            picked.append(best)
            left.remove(best)
        return picked

    spreadLights = pick_spread(tLights, 4)

    # TLs sit off the road, snap them back onto a driving lane wp
    tlRoadLocs = []
    for tl in spreadLights:
        tlPos = tl.get_location()
        wp = cmap.get_waypoint(tlPos, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        tlRoadLocs.append(wp.transform.location)

    startTf = min(allSpawns,
        key=lambda sp: sp.location.distance(tlRoadLocs[0]))

    pathTf = [startTf]
    pathLocs = [startTf.location] + tlRoadLocs

    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "hero")
    car = world.try_spawn_actor(bp, startTf)
    if car is None:
        cands = sorted(allSpawns,
            key=lambda sp: sp.location.distance(tlRoadLocs[0]))
        for c in cands[:5]:
            car = world.try_spawn_actor(bp, c)
            if car is not None:
                pathLocs[0] = c.location
                break
    if car is None:
        raise RuntimeError("Couldnt spawn car.")
    world.tick()

    rSpawns = allSpawns[:]
    random.shuffle(rSpawns)

    vehBps = [bp for bp in world.get_blueprint_library().filter("vehicle")
              if int(bp.get_attribute("number_of_wheels")) == 4]

    npcs = []
    for sp in rSpawns:
        if len(npcs) >= 25:
            break
        if sp.location.distance(car.get_location()) < 10.0:
            continue
        nbp = random.choice(vehBps)
        nbp.set_attribute("role_name", "npc")
        if nbp.has_attribute("color"):
            nbp.set_attribute("color",
                random.choice(nbp.get_attribute("color").recommended_values))
        n = world.try_spawn_actor(nbp, sp)
        if n:
            n.set_autopilot(True, TM_PORT)
            npcs.append(n)
    world.tick()

    wBps = world.get_blueprint_library().filter("walker.pedestrian.*")
    wCtrlBp = world.get_blueprint_library().find("controller.ai.walker")
    navPt = world.get_random_location_from_navigation

    peds = []
    ctrls = []
    for _ in range(25):
        loc = navPt()
        if loc is None:
            continue
        p = world.try_spawn_actor(
            random.choice(wBps), carla.Transform(loc))
        if p is None:
            continue
        ct = world.try_spawn_actor(
            wCtrlBp, carla.Transform(), attach_to=p)
        if ct is None:
            p.destroy()
            continue
        peds.append(p)
        ctrls.append(ct)

    world.tick()
    for ct in ctrls:
        ct.start()
        d = navPt()
        if d:
            ct.go_to_location(d)
        ct.set_max_speed(1.0 + random.random())
    world.tick()

    camBp = world.get_blueprint_library().find("sensor.camera.rgb")
    camBp.set_attribute("image_size_x", str(W))
    camBp.set_attribute("image_size_y", str(H))
    camBp.set_attribute("fov", "90")

    cam = world.spawn_actor(
        camBp,
        carla.Transform(carla.Location(x=-8.0, z=4.0),
                        carla.Rotation(pitch=-15.0)),
        attach_to=car)

    camBuf = [None]

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((H, W, 4))[:, :, :3][:, :, ::-1].copy()
        camBuf[0] = pygame.surfarray.make_surface(arr.swapaxes(0, 1))

    cam.listen(on_image)

    colBp = world.get_blueprint_library().find("sensor.other.collision")
    colSensor = world.spawn_actor(colBp, carla.Transform(), attach_to=car)
    colLog = []
    violPending = [None]

    def on_collision(event):
        other = event.other_actor
        imp = event.normal_impulse
        mag = (imp.x**2 + imp.y**2 + imp.z**2) ** 0.5
        colLog.append({
            "frame": event.frame,
            "other": other.type_id,
            "strength": mag,
            "pos": event.transform.location,
        })
        if violPending[0] is None:
            violPending[0] = f"collision with {other.type_id} mag {mag:.1f}"

    colSensor.listen(on_collision)

    laneBp = world.get_blueprint_library().find("sensor.other.lane_invasion")
    laneSensor = world.spawn_actor(laneBp, carla.Transform(), attach_to=car)
    laneLog = []

    def on_lane(event):
        marks = [str(m.type) for m in event.crossed_lane_markings]
        bad = [m for m in marks
               if "Solid" in m or "Yellow" in m or "Double" in m]
        laneLog.append({
            "frame": event.frame,
            "markings": marks,
            "illegal": bool(bad),
        })
        if bad and violPending[0] is None:
            violPending[0] = f"lane viol {marks}"

    laneSensor.listen(on_lane)


    def buildRoute(locs, step=2.0):
        dense = []
        for i in range(len(locs) - 1):
            origin = cmap.get_waypoint(locs[i],
                                       project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
            target = locs[i + 1]
            wp = origin
            while True:
                loc = wp.transform.location
                if not dense or loc.distance(dense[-1]) > 0.5:
                    dense.append(loc)
                if loc.distance(target) < step * 2:
                    break
                nxt = wp.next(step)
                if not nxt:
                    break
                wp = min(nxt, key=lambda w: w.transform.location.distance(target))
        if dense:
            dense.append(locs[-1])
        return dense

    def goAutopilot(idx):
        remaining = pathLocs[idx:]
        route = buildRoute(remaining)
        tm.set_path(car, route)
        tm.set_desired_speed(car, 30)
        car.set_autopilot(True, TM_PORT)

    while camBuf[0] is None:
        world.tick()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
        screen.fill((20, 20, 20))
        drawTxt(screen, font, "Loading", (W // 2 - 110, H // 2 - 10))
        pygame.display.flip()
        clk.tick(60)

    screen.blit(camBuf[0], (0, 0))
    drawTxt(screen, font, "Ready press key to start",
            (W // 2 - 215, H - 48), (255, 220, 0))
    pygame.display.flip()

    wait = True
    while wait:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type in (pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN):
                wait = False
        clk.tick(60)

    goAutopilot(1)

    wpIdx = 1
    endReason = None  # set when loop exits - violation text or "completed"

    fLog = []
    fActive = False
    # fType = ['bit_flip']  # (Alejandro) default bit flip for testing
    fType = []  # pick_fault sets real value before first use
    fCompound = False
    fEndTime = 0.0
    fNextTime = (time.time() + random.uniform(faultIntMin, faultIntMax)
                 if faultOn else float("inf"))

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt

        world.tick()

        if camBuf[0] is not None:
            screen.blit(camBuf[0], (0, 0))
        else:
            screen.fill((20, 20, 20))

        pos = car.get_location()
        spd = car.get_velocity().length() * 3.6
        dist = pos.distance(pathLocs[wpIdx])
        # print("DEBUG speed", spd)

        if violPending[0] is not None:
            reason = violPending[0]
            endReason = "violation - " + reason

            car.set_autopilot(False, TM_PORT)
            car.apply_control(carla.VehicleControl(
                throttle=0.0, brake=1.0, hand_brake=True))

            deadline = time.time() + 3.0
            while time.time() < deadline:
                world.tick()
                if camBuf[0] is not None:
                    screen.blit(camBuf[0], (0, 0))
                else:
                    screen.fill((20, 20, 20))

                ov = pygame.Surface((W, 120), pygame.SRCALPHA)
                ov.fill((180, 0, 0, 180))
                screen.blit(ov, (0, H // 2 - 60))
                drawTxt(screen, font, "violation - stopping", (W // 2 - 100, H // 2 - 40), (255, 255, 255))
                drawTxt(screen, font, reason, (W // 2 - 100, H // 2), (255, 220, 100))
                pygame.display.flip()
                clk.tick(60)
                for event in pygame.event.get():
                    if event.type in (pygame.QUIT, pygame.KEYDOWN):
                        break
            break

        now = time.time()
        if faultOn:
            if fActive:
                ctrl = car.get_control()
                for ft in fType:
                    apply_single_fault(ctrl, ft)
                car.apply_control(ctrl)

                if now >= fEndTime:
                    fActive = False
                    car.set_autopilot(True, TM_PORT)
                    fNextTime = now + random.uniform(faultIntMin, faultIntMax)

            elif now >= fNextTime:
                fType, fCompound = pick_fault()
                if fType:
                    dur = random.uniform(faultDurMin, faultDurMax)
                    fEndTime = now + dur
                    fActive = True
                    frame = world.get_snapshot().frame
                    tag = "+".join(fType)
                    if fCompound:
                        tag = "compound(" + tag + ")"
                    fLog.append({"frame": frame, "type": tag,
                                 "duration": dur, "compound": fCompound,
                                 "parts": list(fType)})
                    car.set_autopilot(False, TM_PORT)

        drawTxt(screen, font, "spd %.0f km/h" % spd, (12, 12))
        drawTxt(screen, font, f"xy {pos.x:.0f},{pos.y:.0f}", (12, 36))
        drawTxt(screen, font, "wp %d / %d" % (wpIdx - 1, len(pathLocs) - 1), (12, 60))
        drawTxt(screen, font, "dist %.1fm" % dist, (12, 85))

        cc = len(colLog)
        lc = sum(1 for e in laneLog if e["illegal"])
        fc = len(fLog)
        drawTxt(screen, font,
                f"col {cc}  lane viol {lc}  faults {fc}",
                (12, H - 36),
                (255, 80, 80) if (cc + lc) > 0
                else (180, 255, 180))

        if faultOn and fActive:
            tag = "+".join(fType)
            if fCompound:
                tag = "compound(" + tag + ")"
            drawTxt(screen, font,
                    f"fault: {tag}",
                    (W // 2 - 100, H - 64),
                    (255, 40, 40) if fCompound else (255, 100, 0))

        if dist < reachDist:
            if wpIdx == len(pathLocs) - 1:
                endReason = "completed"
                car.set_autopilot(False, TM_PORT)
                car.apply_control(carla.VehicleControl(
                    throttle=0.0, brake=1.0, hand_brake=True))
                drawTxt(screen, font, "path done",
                        (W // 2 - 60, H // 2 - 10), (255, 220, 0))
                pygame.display.flip()
                break
            else:
                wpIdx += 1

        pygame.display.flip()
        clk.tick(60)

    # summary + dump logs to csv so i can chart them later
    stamp = int(time.time())

    print()
    print("== summary ==")
    print("ended", endReason)
    print("wp reached", wpIdx, "of", len(pathLocs) - 1)
    print("col", len(colLog), "/ lane", sum(1 for e in laneLog if e['illegal']),
          "/ faults", len(fLog))
    for e in colLog:
        print(" ", e['frame'], e['other'], round(e['strength'], 1))
    for e in laneLog:
        if e["illegal"]:
            print(" ", e['frame'], e['markings'])
    for e in fLog:
        print(" ", e['frame'], e['type'], "dur", round(e['duration'], 2))

    # csv dumps - one file per log type, timestamped so runs dont overwrite
    with open(f"collisions_{stamp}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "other", "strength", "x", "y", "z"])
        for e in colLog:
            w.writerow([e['frame'], e['other'], e['strength'],
                        e['pos'].x, e['pos'].y, e['pos'].z])

    with open(f"lane_{stamp}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "illegal", "markings"])
        for e in laneLog:
            w.writerow([e['frame'], e['illegal'], "|".join(e['markings'])])

    with open(f"faults_{stamp}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "type", "duration", "compound", "parts"])
        for e in fLog:
            w.writerow([e['frame'], e['type'], round(e['duration'], 3),
                        e['compound'], "|".join(e['parts'])])

    print(f"wrote collisions_{stamp}.csv, lane_{stamp}.csv, faults_{stamp}.csv")

except KeyboardInterrupt:
    print("\nAborted.")

except Exception as e:
    showErr(screen, font, clk, traceback.format_exc())

finally:
    # cleanup - wrap each in try because if something never got spawned
    # it errors out. had this blow up too many times
    try:
        cam.stop()
        cam.destroy()
    except:
        pass
    try:
        colSensor.destroy()
    except:
        pass
    try:
        laneSensor.destroy()
    except:
        pass
    try:
        car.destroy()
    except:
        pass
    try:
        for ct in ctrls:
            ct.stop()
    except:
        pass
    try:
        for n in npcs:
            n.destroy()
        for ct in ctrls:
            ct.destroy()
        for p in peds:
            p.destroy()
    except:
        pass
    # put sync back or the next run loads a frozen world
    try:
        s = world.get_settings()
        s.synchronous_mode = False
        s.fixed_delta_seconds = None
        world.apply_settings(s)
    except:
        pass
    world.tick()
    pygame.quit()
    sys.exit(0)
