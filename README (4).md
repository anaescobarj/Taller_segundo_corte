# Taller Segundo Corte — Real-to-Sim

Taller de la asignatura de Mecatrónica (UMNG) que integra hardware real (ESP32) con simulación física (PyBullet), en tres partes independientes.

## Arquitectura general

En cada parte, una consola de mandos física basada en **ESP32** lee sensores analógicos/digitales (joystick, potenciómetro, botón) y los transmite por **puerto serial (USB)** a un script de **Python** en la PC. Ese script usa **PyBullet** para simular el robot correspondiente en tiempo real, reflejando el movimiento del hardware físico en el mundo simulado ("real-to-sim").

```
[ Joystick / Potenciómetro / Botón ]
              |
              v
        [ ESP32 ]  --- Serial USB (115200 baud) --->  [ Script Python ]
                                                              |
                                                              v
                                                     [ Simulación PyBullet ]
```

## Estructura del repositorio

```
Taller_segundo_corte/
├── gym-pybullet-drones/        # Parte a) — control de dron
│   └── drones_env/             # entorno virtual (Python 3.12)
├── pybullet_robots/             # Partes b) y c) — brazo Baxter y movilidad Atlas
│   ├── data/                    # modelos URDF (baxter_common, atlas, etc.)
│   ├── baxter_ik_demo.py        # utilidades de IK para Baxter (del repo original)
│   ├── esp32_baxter_ik.py       # Parte b) — control del brazo Baxter
│   └── atlas_esp32.py           # Parte c) — movilidad de Atlas
├── robots_env/                  # entorno virtual (Python 3.12) para b) y c)
└── README.md
```

## Requisitos

- Windows con Python 3.12 (entornos virtuales con `venv`)
- Arduino IDE (o `arduino-cli`) para programar la ESP32
- Librerías Python: `pybullet`, `pybullet_data`, `numpy`, `opencv-python`, `pyserial`

Instalación dentro de cada entorno virtual:

```powershell
pip install pybullet numpy opencv-python pyserial --break-system-packages
```

---

## Parte a) — Control de dron con ESP32

Consola de mandos ESP32 para mover un dron simulado entre 3 puntos, usando el repositorio [`gym-pybullet-drones`](https://github.com/utiasDSL/gym-pybullet-drones).

**Video de funcionamiento:** [Ver en Google Drive](https://drive.google.com/file/d/1y6PPhtMNQovSFuN5XUMPjvWE4Eb1wX3t/view?usp=sharing)

**Hardware:** _(completar: joystick/botones usados y su función)_

**Cómo correrlo:**

```powershell
cd gym-pybullet-drones
.\drones_env\Scripts\Activate.ps1
python <script_dron>.py
```

![Montaje físico de la consola ESP32 para el dron](images/montaje_dron.png)

_(agregar aquí una captura de la simulación del dron: `![Dron en PyBullet](images/dron.png)`)_

---

## Parte b) — Brazo Baxter con cinemática inversa (IK)

Consola de mandos ESP32 (joystick analógico de dos ejes + potenciómetro + botón) para mover el brazo del robot Baxter en simulación, usando cinemática inversa (`accurateIK`, del repo [`pybullet_robots`](https://github.com/erwincoumans/pybullet_robots)) y agarrar/soltar un objeto (cubo).

**Video de funcionamiento:** [Ver en Google Drive](https://drive.google.com/file/d/1GWLRM26hFdsxEiUiFdUt3AabZTIxaHjY/view?usp=sharing)

**Hardware:**
- Joystick analógico (módulo tipo KY-023): ejes X/Y → posición X/Y del efector final; botón del joystick → abrir/cerrar la pinza
- Potenciómetro: eje Z (altura) del efector final

**Protocolo serial (ESP32 → PC):** líneas de texto `x <valor>`, `y <valor>`, `z <valor>`, y `g` cuando se presiona el botón (alterna abrir/cerrar pinza).

![Montaje físico de la consola ESP32 con joystick y potenciómetro para el Baxter](images/montaje_baxter.png)

**Cómo correrlo:**

```powershell
cd pybullet_robots
..\robots_env\Scripts\Activate.ps1
python esp32_baxter_ik.py
```

1. Mueve el joystick/potenciómetro para posicionar el efector final del brazo cerca del objeto.
2. Presiona el botón para cerrar la pinza cuando estés a menos de 8 cm del objeto (se crea una restricción física que "pega" el objeto a la mano).
3. Presiona el botón de nuevo para soltarlo.

**Problema conocido:** el potenciómetro usado para el eje Z puede requerir revisión de cableado (conexión floja del wiper al pin ADC) — verificar con un sketch de prueba (`analogRead` directo) si los valores no varían al girarlo.

![Brazo Baxter posicionado junto al objeto a agarrar](images/baxter_agarre.png)

---

## Parte c) — Movilidad real de Atlas con cámaras sintéticas

Consola de mandos ESP32 (mismo joystick de la parte b) para desplazar la base del robot humanoide **Atlas** (modelo `atlas/atlas_v4_with_multisense.urdf`, incluido en el repo `pybullet_robots`) por el escenario simulado, con tres cámaras sintéticas (RGB, profundidad, segmentación) observando la escena en tiempo real.

**Video de funcionamiento:** [Ver en Google Drive](https://drive.google.com/file/d/1zXPqKCPpUMop8Oft9iahWHYt0ATbMfFB/view?usp=sharing)

**Hardware:**
- Joystick analógico: eje Y → avance/retroceso; eje X → giro (izquierda/derecha)
- Botón: reinicia a Atlas a su posición inicial

**Diseño:**
- Atlas se carga con `useFixedBase=True` y todos sus joints se fijan con `POSITION_CONTROL` para que no colapse por gravedad (no se simula marcha bípeda real; la base se traslada/rota directamente según el joystick, como un robot móvil).
- Al iniciar, el script calibra automáticamente el "centro" del joystick (2 segundos sin tocarlo) para compensar el offset natural del potenciómetro en reposo, y aplica además una zona muerta (deadzone) para evitar drift.
- Las tres cámaras sintéticas (RGB, profundidad, segmentación por color) están fijas, observando la escena mientras Atlas se mueve dentro del campo de visión.

![Montaje físico de la consola ESP32 con joystick para Atlas](images/montaje_atlas.png)

**Cómo correrlo:**

```powershell
cd pybullet_robots
..\robots_env\Scripts\Activate.ps1
python atlas_esp32.py
```

Al arrancar, no toques el joystick durante los 2 segundos de calibración inicial (mensaje en consola). Luego, muévelo para desplazar y girar a Atlas.

![Atlas cargado en la simulación](images/atlas_cargado.png)

![Cámaras sintéticas (RGB, profundidad, segmentación) observando a Atlas](images/atlas_camaras.png)

---

## Autora

Ana — Ingeniería Mecatrónica, Universidad Militar Nueva Granada (UMNG)
