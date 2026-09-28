from time import sleep
import threading
import contextlib
import os
import pybullet as p
import pybullet_data
import numpy as np
import cv2

from baxter_ik_demo import setUpWorld, getJointRanges, accurateIK, setMotors
import baxter_ik_demo

estado = {"x": 0.2, "y": 0.0, "z": -0.1, "pinza_cerrada": False}
lock = threading.Lock()

def leer_teclado():
    global estado
    while True:
        comando = input("Comando (x/y/z <valor>, o 'g' para pinza): ").strip().lower()
        partes = comando.split()
        with lock:
            if len(partes) == 2 and partes[0] in ("x", "y", "z"):
                try:
                    estado[partes[0]] = float(partes[1])
                    print(f"Nuevo {partes[0]} = {estado[partes[0]]}")
                except ValueError:
                    print("Valor no válido.")
            elif comando == "g":
                estado["pinza_cerrada"] = not estado["pinza_cerrada"]
                print("Pinza:", "CERRADA" if estado["pinza_cerrada"] else "ABIERTA")
            else:
                print("Comando no reconocido.")

hilo = threading.Thread(target=leer_teclado, daemon=True)
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

# --- Configuración de la cámara sintética ---
ANCHO_CAM, ALTO_CAM = 320, 240
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[1.2, 0.0, 0.6],
    cameraTargetPosition=[0.2, 0.0, -0.1],
    cameraUpVector=[0, 0, 1],
)
proj_matrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=ANCHO_CAM / ALTO_CAM, nearVal=0.1, farVal=3.0
)

contador = 0

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

    # --- Actualizar cámaras cada pocos pasos (por rendimiento) ---
    contador += 1
    if contador % 4 == 0:
        img = p.getCameraImage(
            ANCHO_CAM, ALTO_CAM, view_matrix, proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        rgb = np.reshape(img[2], (ALTO_CAM, ANCHO_CAM, 4))[:, :, :3].astype(np.uint8)
        depth_buffer = np.reshape(img[3], (ALTO_CAM, ANCHO_CAM))
        seg_mask = np.reshape(img[4], (ALTO_CAM, ANCHO_CAM))

        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        depth_vis = (depth_buffer * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_BONE)

        seg_vis = (seg_mask.astype(np.float32) /
                   (seg_mask.max() + 1e-6) * 255).astype(np.uint8)
        seg_vis = cv2.applyColorMap(seg_vis, cv2.COLORMAP_JET)

        cv2.imshow("Camara RGB", rgb_bgr)
        cv2.imshow("Camara Profundidad", depth_vis)
        cv2.imshow("Camara Segmentacion", seg_vis)
        cv2.waitKey(1)

    sleep(1. / 240.)