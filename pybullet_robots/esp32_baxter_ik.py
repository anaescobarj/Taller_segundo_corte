from time import sleep
import threading
import contextlib
import os
import pybullet as p
import pybullet_data
import numpy as np
import serial

from baxter_ik_demo import setUpWorld, getJointRanges, accurateIK, setMotors
import baxter_ik_demo

PUERTO_ESP32 = "COM5"  # cámbialo por tu puerto real
BAUDIOS = 115200

estado = {"x": 0.2, "y": 0.0, "z": -0.1, "pinza_cerrada": False}
lock = threading.Lock()

def leer_serial():
    global estado
    ser = serial.Serial(PUERTO_ESP32, BAUDIOS, timeout=1)
    print(f"[ESP32] Puerto {PUERTO_ESP32} abierto correctamente.")
    while True:
        linea = ser.readline().decode(errors="ignore").strip().lower()
        if not linea:
            continue
        partes = linea.split()
        with lock:
            if len(partes) == 2 and partes[0] in ("x", "y", "z"):
                try:
                    estado[partes[0]] = float(partes[1])
                except ValueError:
                    pass
            elif linea == "g":
                estado["pinza_cerrada"] = not estado["pinza_cerrada"]
                print("Pinza:", "CERRADA" if estado["pinza_cerrada"] else "ABIERTA")

hilo = threading.Thread(target=leer_serial, daemon=True)
hilo.start()

guiClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(2., 180, 0., [0.52, 0.2, np.pi/4.])

baxterId, endEffectorId = setUpWorld()
baxter_ik_demo.baxterId = baxterId

lowerLimits, upperLimits, jointRanges, restPoses = getJointRanges(baxterId, includeFixed=False)

posicion_objeto_inicial = [0.2, 0.0, -0.1]
objetoId = p.loadURDF("cube_small.urdf", posicion_objeto_inicial, globalScaling=1.0)

constraint_id = None

p.addUserDebugText("OBJETO", posicion_objeto_inicial, textColorRGB=[0, 1, 0], textSize=1.2)

sleep(1.)

_devnull = open(os.devnull, "w")

while True:
    p.stepSimulation()

    if constraint_id is None:
        p.resetBasePositionAndOrientation(objetoId, posicion_objeto_inicial, [0, 0, 0, 1])

    with lock:
        targetPosition = [estado["x"], estado["y"], estado["z"]]
        pinza_cerrada = estado["pinza_cerrada"]

    with contextlib.redirect_stdout(_devnull):
        jointPoses = accurateIK(baxterId, endEffectorId, targetPosition,
                                 lowerLimits, upperLimits, jointRanges, restPoses,
                                 useNullSpace=True)
    setMotors(baxterId, jointPoses)

    endEffectorPos = p.getLinkState(baxterId, endEffectorId)[4]
    objetoPos, _ = p.getBasePositionAndOrientation(objetoId)
    distancia = np.linalg.norm(np.array(endEffectorPos) - np.array(objetoPos))

    if pinza_cerrada and constraint_id is None and distancia < 0.08:
        constraint_id = p.createConstraint(
            parentBodyUniqueId=baxterId,
            parentLinkIndex=endEffectorId,
            childBodyUniqueId=objetoId,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        print(">>> Objeto agarrado")

    if not pinza_cerrada and constraint_id is not None:
        p.removeConstraint(constraint_id)
        constraint_id = None
        print(">>> Objeto soltado")

    sleep(1. / 240.)