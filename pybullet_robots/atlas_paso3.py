import os
import threading
from time import sleep

import pybullet as p
import pybullet_data
import numpy as np
import cv2

# --- Estado compartido (por ahora con teclado) ---
estado = {"avance": 0.0, "giro": 0.0}
lock = threading.Lock()

def leer_teclado():
    global estado
    while True:
        comando = input("Comando (a <valor>=avance, g <valor>=giro): ").strip().lower()
        partes = comando.split()
        with lock:
            if len(partes) == 2 and partes[0] in ("a", "g"):
                try:
                    valor = float(partes[1])
                    clave = {"a": "avance", "g": "giro"}[partes[0]]
                    estado[clave] = valor
                    print(f"Nuevo {clave} = {valor}")
                except ValueError:
                    print("Valor no válido.")
            else:
                print("Comando no reconocido.")

hilo = threading.Thread(target=leer_teclado, daemon=True)
hilo.start()

p.connect(p.GUI)
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

p.resetDebugVisualizerCamera(4.0, 45, -20, [0, 0, 0.9])

print(f"Atlas cargado con ID: {atlasId}")
print(f"Número de joints: {p.getNumJoints(atlasId)}")

# --- Cámara sintética FIJA, observando la escena ---
ANCHO_CAM, ALTO_CAM = 320, 240

view_matrix = p.computeViewMatrix(
    cameraEyePosition=[4.0, 0.0, 1.5],
    cameraTargetPosition=[0, 0, 0.9],
    cameraUpVector=[0, 0, 1],
)
proj_matrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=ANCHO_CAM / ALTO_CAM, nearVal=0.1, farVal=8.0
)

pos_actual = list(posicion_inicial)
yaw_actual = 0.0
contador = 0

while True:
    p.stepSimulation()

    with lock:
        avance = estado["avance"]
        giro = estado["giro"]

    # Mover la base de Atlas como robot diferencial (teletransportando)
    yaw_actual += giro * 0.02
    pos_actual[0] += avance * np.cos(yaw_actual) * 0.01
    pos_actual[1] += avance * np.sin(yaw_actual) * 0.01

    orn = p.getQuaternionFromEuler([0, 0, yaw_actual])
    p.resetBasePositionAndOrientation(atlasId, pos_actual, orn)

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
        depth_vis = cv2.applyColorMap((depth_buffer * 255).astype(np.uint8), cv2.COLORMAP_BONE)

        seg_max = seg_mask.max()
        if seg_max > 0:
            seg_vis = cv2.applyColorMap(
                (seg_mask.astype(np.float32) / seg_max * 255).astype(np.uint8),
                cv2.COLORMAP_JET
            )
        else:
            seg_vis = np.zeros((ALTO_CAM, ANCHO_CAM, 3), dtype=np.uint8)

        cv2.imshow("Camara RGB", rgb_bgr)
        cv2.imshow("Camara Profundidad", depth_vis)
        cv2.imshow("Camara Segmentacion", seg_vis)
        cv2.waitKey(1)

    sleep(1. / 240.)