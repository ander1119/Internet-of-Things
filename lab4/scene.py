import lgsvl
import random
import time

from threading import Thread, Lock


sim = lgsvl.Simulator(address = "localhost", port = 8181)
try:
    sim.reset()
except:
    pass

# Inits simulation.
sim.load(scene = "Shalun", seed = 650387)


# Chooses ego vehicles.
state = lgsvl.AgentState()
state.transform.position = lgsvl.Vector(-9, 0, 43)
state.transform.rotation.y = 159
ego = sim.add_agent(name = "Lexus2016RXHybrid (Autoware)",
        agent_type = lgsvl.AgentType.EGO, state = state)

# Connects to Autoware Ros2 bridge.
ego.connect_bridge("127.0.0.1", 9090)

# Add NPC vehicle
state = lgsvl.AgentState()
state.transform.position = lgsvl.Vector(-5, 0, 18.8+5*2.69)
state.transform.rotation.y = 159
npc = sim.add_agent(name = "Sedan", agent_type = lgsvl.AgentType.NPC, state = state)
waypoints = []
speed = 10/3.6
speed_range = (5/3.6, 40/3.6)
acc_range = (5, 10)
for i in range(-4, 10):
    acc = random.uniform(*acc_range)
    if 4*(speed**2)+8*acc < 0:
        end_speed = 0
    else:
        t = (-2*speed+(4*(speed**2)+4*acc)**0.5)/(2*acc)
        end_speed = speed+acc*t
    if end_speed < speed_range[0]:
        end_speed = speed_range[0]
    elif end_speed > speed_range[1]:
        end_speed = speed_range[1]
    speed = end_speed
    waypoints.append(lgsvl.DriveWaypoint(lgsvl.Vector(i, 0, 18.8-i*2.69), speed, lgsvl.Vector(0, 159, 0), 0, True, 0))
acc_range = (-10, -5)
for i in range(10, 24):
    acc = random.uniform(*acc_range)
    if 4*(speed**2)+8*acc < 0:
        end_speed = 0
    else:
        t = (-2*speed+(4*(speed**2)+4*acc)**0.5)/(2*acc)
        end_speed = speed+acc*t
    if end_speed < speed_range[0]:
        end_speed = speed_range[0]
    elif end_speed > speed_range[1]:
        end_speed = speed_range[1]
    speed = end_speed
    waypoints.append(lgsvl.DriveWaypoint(lgsvl.Vector(i, 0, 18.8-i*2.69), speed, lgsvl.Vector(0, 159, 0)))

# Check distance at every waypoint.
total_score = 0.0
def on_waypoint(agent, index):
    global total_distance, total_score
    print("ckeckpoint %02d reached" % index)
    npc_state = npc.state
    ego_state = ego.state
    distance = (((npc_state.position.x - ego_state.position.x)**2 +
                    (npc_state.position.z - ego_state.position.z)**2) ** 0.5) - 4.7 
    if distance < 3.0:
        score = 40
    elif distance < 4.0:
        score = 65
    elif distance < 5.0:
        score = 75
    elif distance < 6.0:
        score = 90
    elif distance <= 7.0:
        score = 100
    elif distance <= 8.0:
        score = 90
    elif distance <= 9.0:
        score = 80 
    elif distance <= 10.0:
        score = 65
    elif distance <= 11.0:
        score = 40
    elif distance <= 12.0:
        score = 20
    else:
        score = 0
    total_score += score
    print ("distance: %f m, score: %f" % (distance, score))
    if index == len(waypoints)-1:
        sim.stop()
        print ('Simulation stoped. Final score: %f' % (total_score / len(waypoints)))
        exit(0)
npc.follow(waypoints, loop=False)
npc.on_waypoint_reached(on_waypoint)

failed = False

def on_collision(agent1, agent2, contact):
    global failed
    print("Collision happened. Testing failed")
    failed = True
    sim.stop()
ego.on_collision(on_collision)
npc.on_collision(on_collision)

# Starts simulation.
state = 0
throttle = 0.5
while not failed:
    sim.run()

