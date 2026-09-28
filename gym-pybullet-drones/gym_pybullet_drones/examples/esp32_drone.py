import time
import threading
import contextlib
import os
import numpy as np
import serial

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

# --- Puntos A, B, C (x, y, z en metros) ---
PUNTOS = {
    "A": np.array([0.0, 0.0, 1.0]),
    "B": np.array([0.5, 0.5, 1.0]),
    "C": np.array([-0.5, 0.5, 1.5]),
}

punto_objetivo = PUNTOS["A"]
lock = threading.Lock()

# --- Configura aquí tu puerto serial ---
PUERTO_ESP32 = "COM5"  # cámbialo por tu puerto real
BAUDIOS = 115200

def leer_serial():
    """Escucha la ESP32 en un hilo aparte, sin bloquear la simulación."""
    global punto_objetivo
    ser = serial.Serial(PUERTO_ESP32, BAUDIOS, timeout=1)
    print(f"[ESP32] Puerto {PUERTO_ESP32} abierto correctamente, esperando datos...")
    while True:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea:
            print(f"[ESP32] Recibido crudo: '{linea}'")
        if linea in PUNTOS:
            with lock:
                punto_objetivo = PUNTOS[linea]
            print(f"[ESP32] >>> Nuevo destino: {linea} -> {PUNTOS[linea]}")

hilo = threading.Thread(target=leer_serial, daemon=True)
hilo.start()

# --- Configuración del dron y la simulación ---
DRONE = DroneModel("cf2x")
SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48
DURATION_SEC = 120  # duración total de la simulación

INIT_XYZS = np.array([[0.0, 0.0, 0.1]])
INIT_RPYS = np.array([[0.0, 0.0, 0.0]])

env = CtrlAviary(
    drone_model=DRONE,
    num_drones=1,
    initial_xyzs=INIT_XYZS,
    initial_rpys=INIT_RPYS,
    physics=Physics("pyb"),
    neighbourhood_radius=10,
    pyb_freq=SIMULATION_FREQ_HZ,
    ctrl_freq=CONTROL_FREQ_HZ,
    gui=True,
    record=False,
    obstacles=False,
    user_debug_gui=False,
)

ctrl = DSLPIDControl(drone_model=DRONE)

action = np.zeros((1, 4))
START = time.time()
_devnull = open(os.devnull, "w")

for i in range(0, int(DURATION_SEC * env.CTRL_FREQ)):
    obs, reward, terminated, truncated, info = env.step(action)

    with lock:
        objetivo_actual = punto_objetivo

    action[0, :], _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=obs[0],
        target_pos=objetivo_actual,
        target_rpy=np.array([0.0, 0.0, 0.0]),
    )

    # Silenciamos los prints automáticos de env.render() para que no
    # entierren los mensajes de la ESP32
    with contextlib.redirect_stdout(_devnull):
        env.render()

    sync(i, START, env.CTRL_TIMESTEP)

env.close()