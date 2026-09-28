import os
import pybullet as p
import pybullet_data
from time import sleep

p.connect(p.GUI)

# Ruta de pybullet_data (pip) + ruta local del repo pybullet_robots (donde está Atlas)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
REPO_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
p.setAdditionalSearchPath(REPO_DATA)

p.resetSimulation()
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

posicion_inicial = [0, 0, 0.9]
orientacion_inicial = p.getQuaternionFromEuler([0, 0, 0])

atlasId = p.loadURDF(
    "atlas/atlas_v4_with_multisense.urdf",
    posicion_inicial,
    orientacion_inicial,
    useFixedBase=True
)

p.resetDebugVisualizerCamera(3.0, 45, -20, posicion_inicial)

print(f"Atlas cargado con ID: {atlasId}")
print(f"Número de joints: {p.getNumJoints(atlasId)}")

while True:
    p.stepSimulation()
    sleep(1. / 240.)