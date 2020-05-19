import lgsvl
import random
import time

from threading import Thread, Lock

DIS_THROSHOLD = 3
TIME_LIMIT = 300

sim = lgsvl.Simulator(address = "localhost", port = 8181)
try:
    sim.reset()
except:
    pass

# Inits simulation.
sim.load(scene = "Shalun", seed = 650387)


# Chooses ego vehicles.
state = lgsvl.AgentState()
state.transform.position = lgsvl.Vector(-7, 0, 76)
state.transform.rotation.y = 190
ego = sim.add_agent(name = "Lexus2016RXHybrid (Autoware)",
        agent_type = lgsvl.AgentType.EGO, state = state)

# Connects to Autoware Ros2 bridge.
ego.connect_bridge("127.0.0.1", 9090)

failed = False

def on_collision(agent1, agent2, contact):
    global failed
    print("Collision happened. Testing failed")
    failed = True
    sim.stop()
#ego.on_collision(on_collision)

#checkPoints
class CheckPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.dis = 10000000
    
    def update(self, x, y):
        dis = (((x - self.x) ** 2) + ((y - self.y) ** 2)) ** 0.5
        if dis < self.dis:
            self.dis = dis

    def getDis(self):
        return self.dis

checkPoints = []
with open('waypoints.csv') as f:
    line = f.readline()
    while line:
        x, y = [float(v) for v in line.split(',')]
        checkPoints.append(CheckPoint(x, y))
        line = f.readline()

currentCheckPoint = 0
time_passed = 0

# Starts simulation.
while not failed:
    sim.run(0.1)
    if currentCheckPoint > 0:
        time_passed += 0.1
    ego_state = ego.state
    for i in range(-2, 1):
        if currentCheckPoint + i >= 0 and currentCheckPoint + i < len(checkPoints):
            checkPoints[currentCheckPoint + i].update(ego_state.position.x, ego_state.position.z)
    if currentCheckPoint < len(checkPoints) and checkPoints[currentCheckPoint].getDis() < DIS_THROSHOLD:
        print ('Waypoint %d passed.' % currentCheckPoint)
        currentCheckPoint += 1

    if currentCheckPoint >= len(checkPoints):
        velocity = (ego_state.velocity.x**2+ego_state.velocity.y**2+ego_state.velocity.z**2)**0.5
        if velocity < 0.1:
            break
    if time_passed > TIME_LIMIT:
        failed = True
        print ('Timeout')

score_total = 0
if not failed:
    for i in range(len(checkPoints)):
        dis = checkPoints[i].getDis()
        if dis <= 0.5:
            score = 100
        elif dis <= 2.0:
            score = (2.0 - dis) * 20.0 / 1.5 + 80.0
        else:
            score = (3.0 - dis) * 20.0 + 60.0
        score_total += score
        print (checkPoints[i].getDis(), score)
final_score = score_total / len(checkPoints) * 0.8
print ('Time passed: %.2f s' % time_passed) 
if time_passed <= 90:
    final_score += 20
elif time_passed <= 180:
    final_score += 10
print ('Final score: %.2f' % final_score)
