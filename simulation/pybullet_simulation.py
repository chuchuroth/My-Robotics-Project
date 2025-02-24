import pybullet as p
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)

# Load plane and robot URDFs
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf")

# Set gravity
p.setGravity(0, 0, -9.81)

# Run simulation for 5 seconds
for _ in range(240):  # 240 steps = 5 seconds at 48 Hz
    p.stepSimulation()
    time.sleep(1./48.)

# Disconnect from the server
p.disconnect()
