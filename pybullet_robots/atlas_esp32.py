import os
import time
import threading
from time import sleep

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import serial

PUERTO_ESP32 = "COM5"   # ajusta si es distinto
BAUDIOS = 115200
DEADZONE = 0.05          # zona muerta pequeña, ya que calibramos el centro real

# --- Estado compartido, actualizado desde el hilo serial ---
estado = {"avance": 0.0, "giro": 0.0}
lock = threading.Lock()
reset_solicitado = threading.Event()

# --- Calibración del centro del joystick ---
offset = {"avance": 0.0, "giro": 0.0}
calibrado = threading.Event()

def leer_esp32():
    try:
        ser = serial.Serial(PUERTO_ESP32, BAUDIOS, timeout=1)
        print(f"[ESP32] Puerto {PUERTO_ESP32} abierto correctamente.")
    except Exception as e:
        print(f"[ESP32] ERROR al abrir el puerto: {e}")
        return

    print(">>> Calibrando centro del joystick, NO LO TOQUES por 2 segundos...")
    muestras_x, muestras_y = [], []
    t_inicio = time.time()
    while time.time() - t_inicio < 2.0:
        try:
            linea = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        partes = linea.split()
        if len(partes) == 2:
            try:
                val = float(partes[1])
                if partes[0] == "x":
                    muestras_x.append(val)
                elif partes[0] == "y":
                    muestras_y.append(val)
            except ValueError:
                pass

    if muestras_x:
        offset["giro"] = sum(muestras_x) / len(muestras_x)
    if muestras_y:
        offset["avance"] = sum(muestras_y) / len(muestras_y)
    calibrado.set()
    print(f">>> Calibración lista. Offset x={offset['giro']:.3f}, y={offset['avance']:.3f}")

    while True:
        try:
            linea = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not linea:
            continue

        partes = linea.split()
        with lock:
            if len(partes) == 2 and partes[0] == "y":
                try:
                    estado["avance"] = float(partes[1]) - offset["avance"]
                except ValueError:
                    pass
            elif len(partes) == 2 and partes[0] == "x":
                try:
                    estado["giro"] = float(partes[1]) - offset["giro"]
                except ValueError:
                    pass
            elif partes[0] == "g":
                reset_solicitado.set()

hilo = threading.Thread(target=leer_esp32, daemon=True)
hilo.start()

print(">>> Esperando calibración del joystick antes de iniciar la simulación...")
calibrado.wait(timeout=5.0)

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

# --- Fijar todos los joints en su posición actual para que no colapsen por gravedad ---
for i in range(p.getNumJoints(atlasId)):
    p.setJointMotorControl2(atlasId, i, p.POSITION_CONTROL, targetPosition=0, force=500)

print(f"Atlas cargado con ID: {atlasId}")
print(f"Número de joints: {p.getNumJoints(atlasId)}")

# --- Cámara sintética FIJA ---
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

    if reset_solicitado.is_set():
        pos_actual = list(posicion_inicial)
        yaw_actual = 0.0
        reset_solicitado.clear()
        print(">>> Atlas reiniciado a posición inicial")

    with lock:
        avance = estado["avance"]
        giro = estado["giro"]

    # --- Zona muerta para evitar drift residual del joystick ---
    if abs(avance) < DEADZONE:
        avance = 0.0
    if abs(giro) < DEADZONE:
        giro = 0.0

    yaw_actual += giro * 0.05
    pos_actual[0] += avance * np.cos(yaw_actual) * 0.03
    pos_actual[1] += avance * np.sin(yaw_actual) * 0.03

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