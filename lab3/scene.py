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
#POSE_LIST = [(20, 0, -35), (19, 0, -32.3), (18, 0, -29.6), (17, 0, -26.9)]
#front_car = random.choice(POSE_LIST)
front_car = (17, 0, -26.9)
state = lgsvl.AgentState()
state.transform.position = lgsvl.Vector(*front_car)
state.transform.rotation.y = 159
npc = sim.add_agent(name = "Sedan", agent_type = lgsvl.AgentType.NPC, state = state)

# Add Other vehicles
OTHER_LIST = [(10, 0, 20, -21), (9, 0, 4, 159), (4, 0, -19, 69), (27, 0, -24, -21)]
state = lgsvl.AgentState()
for car in OTHER_LIST:
    state.transform.position = lgsvl.Vector(*car[:3])
    state.transform.rotation.y = car[3]
    sim.add_agent(name = "Sedan", agent_type = lgsvl.AgentType.NPC, state = state)


failed = False

def on_collision(agent1, agent2, contact):
    print("Collision happened. Testing failed")
    failed = True
ego.on_collision(on_collision)

# Starts simulation.
state = 0
while not failed:
    #sim.lock.acquire()
    sim.run(time_limit = 0.1)
    #sim.lock.release()
    #judge
    s = ego.state
    velocity = (s.velocity.x**2+s.velocity.y**2+s.velocity.z**2)**0.5
    if state == 0:
        if velocity * 3.6 >= 40:
            print('the first stage Passed.')
            state = 1
    if state == 1:
        if velocity * 3.6 <= 5:
            distant = (((s.position.x - front_car[0])**2 + 
                    (s.position.z - front_car[2])**2) ** 0.5) - 6
            print('Vehicle stopped with distant: %0.3f' % distant)
            sim.stop()
            break


