import re
import time
import threading

import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # nötig für 3D

# ----------------- Konfiguration -----------------
PORT = "COM8"
BAUD = 115200
TIMEOUT = 1.0  # Sekunden
# -------------------------------------------------

# Globale Variablen
quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
euler = np.array([0.0, 0.0, 0.0], dtype=float)
lin_accel = np.array([0.0, 0.0, 0.0], dtype=float)
gravity = np.array([0.0, 0.0, 0.0], dtype=float)
calib_status = {"SYS": 0, "G": 0, "A": 0, "M": 0}  # 0-3

quat_lock = threading.Lock()
data_lock = threading.Lock()
running = True

# Regex-Patterns
quat_pattern = re.compile(r"Quaternion/BNO055_(W|X|Y|Z)\[raw\]:\s*([-+0-9.eE]+)")
euler_pattern = re.compile(r"Euler/BNO055_(Heading|Roll|Pitch)\[raw\]:\s*([-+0-9.eE]+)")
linaccel_pattern = re.compile(r"LinearAccel/BNO055_(X|Y|Z)\[raw\]:\s*([-+0-9.eE]+)")
gravity_pattern = re.compile(r"Gravity/BNO055_(X|Y|Z)\[raw\]:\s*([-+0-9.eE]+)")
calib_pattern = re.compile(r"Status:\s*SYS:(\d)\s*G:(\d)\s*A:(\d)\s*M:(\d)")

def serial_reader():
    global quat, euler, lin_accel, gravity, calib_status, running
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    
    # Puffer für alle Daten
    quat_current = {"W": None, "X": None, "Y": None, "Z": None}
    euler_current = {"Heading": None, "Roll": None, "Pitch": None}
    lin_current = {"X": None, "Y": None, "Z": None}
    grav_current = {"X": None, "Y": None, "Z": None}

    while running:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        # Kalibrierungsstatus
        calib_match = calib_pattern.match(line)
        if calib_match:
            sys, g, a, m = map(int, calib_match.groups())
            with data_lock:
                calib_status["SYS"] = sys
                calib_status["G"] = g
                calib_status["A"] = a
                calib_status["M"] = m
            continue

        # Quaternion
        quat_match = quat_pattern.match(line)
        if quat_match:
            axis = quat_match.group(1)
            val = float(quat_match.group(2))
            quat_current[axis] = val
            if all(v is not None for v in quat_current.values()):
                w, x, y, z = [quat_current[k] for k in "WXYZ"]
                q = np.array([w, x, y, z], dtype=float)
                n = np.linalg.norm(q)
                if n > 0:
                    q /= n
                with data_lock:
                    quat = q
                quat_current = {"W": None, "X": None, "Y": None, "Z": None}

        # Euler
        euler_match = euler_pattern.match(line)
        if euler_match:
            axis = euler_match.group(1)
            val = float(euler_match.group(2))
            euler_current[axis] = val
            if all(v is not None for v in euler_current.values()):
                euler_data = np.array([euler_current["Heading"], 
                                     euler_current["Roll"], 
                                     euler_current["Pitch"]], dtype=float)
                with data_lock:
                    euler = euler_data
                euler_current = {"Heading": None, "Roll": None, "Pitch": None}

        # Linear Acceleration
        lin_match = linaccel_pattern.match(line)
        if lin_match:
            axis = lin_match.group(1)
            val = float(lin_match.group(2))
            lin_current[axis] = val
            if all(v is not None for v in lin_current.values()):
                with data_lock:
                    lin_accel = np.array([lin_current["X"], lin_current["Y"], lin_current["Z"]], dtype=float)
                lin_current = {"X": None, "Y": None, "Z": None}

        # Gravity
        grav_match = gravity_pattern.match(line)
        if grav_match:
            axis = grav_match.group(1)
            val = float(grav_match.group(2))
            grav_current[axis] = val
            if all(v is not None for v in grav_current.values()):
                with data_lock:
                    gravity = np.array([grav_current["X"], grav_current["Y"], grav_current["Z"]], dtype=float)
                grav_current = {"X": None, "Y": None, "Z": None}

    ser.close()

def quat_to_rotmat(q):
    """Quaternion zu Rotationsmatrix."""
    w, x, y, z = q
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    R = np.array([
        [ww + xx - yy - zz, 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     ww - xx + yy - zz, 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     ww - xx - yy + zz]
    ])
    return R

def main():
    global running

    # UART-Thread starten
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # Matplotlib-Setup: 2x2 Grid
    plt.ion()
    fig = plt.figure(figsize=(15, 12))
    
    # 1. Quaternion Orientierung (oben links)
    ax_quat = fig.add_subplot(221, projection="3d")
    ax_quat.set_xlim([-1, 1]); ax_quat.set_ylim([-1, 1]); ax_quat.set_zlim([-1, 1])
    ax_quat.set_xlabel("X"); ax_quat.set_ylabel("Y"); ax_quat.set_zlabel("Z")
    ax_quat.set_title("Quaternion Orientierung")
    
    # Weltachsen (grau)
    w_x, = ax_quat.plot([0,1],[0,0],[0,0], '0.7', lw=1)
    w_y, = ax_quat.plot([0,0],[0,1],[0,0], '0.7', lw=1)
    w_z, = ax_quat.plot([0,0],[0,0],[0,1], '0.7', lw=1)
    
    # Sensorachsen (RGB)
    b_x, = ax_quat.plot([0,1],[0,0],[0,0], 'r', lw=3)
    b_y, = ax_quat.plot([0,0],[0,1],[0,0], 'g', lw=3)
    b_z, = ax_quat.plot([0,0],[0,0],[0,1], 'b', lw=3)

    # 2. Euler Vektoren (oben rechts)
    ax_euler = fig.add_subplot(222, projection="3d")
    ax_euler.set_xlim([-400, 400]); ax_euler.set_ylim([-400, 400]); ax_euler.set_zlim([-400, 400])
    ax_euler.set_xlabel("Heading"); ax_euler.set_ylabel("Roll"); ax_euler.set_zlabel("Pitch")
    ax_euler.set_title("Euler Winkel")
    euler_vec, = ax_euler.plot([0,0],[0,0],[0,0], 'm', lw=4, label="Euler")

    # 3. Linear Acceleration (unten links)
    ax_lin = fig.add_subplot(223, projection="3d")
    ax_lin.set_xlim([-10, 10]); ax_lin.set_ylim([-10, 10]); ax_lin.set_zlim([-10, 10])
    ax_lin.set_xlabel("X"); ax_lin.set_ylabel("Y"); ax_lin.set_zlabel("Z")
    ax_lin.set_title("Linear Acceleration (m/s²)")
    lin_vec, = ax_lin.plot([0,0],[0,0],[0,0], 'c', lw=4, label="Lin. Accel")

    # 4. Gravity + Kalibrierung (unten rechts)
    ax_grav_cal = fig.add_subplot(224, projection="3d")
    ax_grav_cal.set_xlim([-10, 10]); ax_grav_cal.set_ylim([-10, 10]); ax_grav_cal.set_zlim([-10, 10])
    ax_grav_cal.set_xlabel("X"); ax_grav_cal.set_ylabel("Y"); ax_grav_cal.set_zlabel("Z")
    ax_grav_cal.set_title("Gravity (m/s²)")
    grav_vec, = ax_grav_cal.plot([0,0],[0,0],[0,0], 'orange', lw=4, label="Gravity")
    
    # Kalibrierungsbalken als Overlay
    calib_ax = fig.add_axes([0.78, 0.15, 0.18, 0.25])  # Positioniert
    sensors = ["SYS", "G", "A", "M"]
    colors = ["red", "orange", "yellow", "green"]
    bars = calib_ax.bar(sensors, [0,0,0,0], color=colors, alpha=0.8)
    calib_ax.set_ylim(0, 3)
    calib_ax.set_ylabel("Calib\n(0-3)")
    calib_ax.grid(True, alpha=0.3)

    try:
        while plt.fignum_exists(fig.number):
            # Alle Daten kopieren
            with data_lock:
                q = quat.copy()
                e = euler.copy()
                la = lin_accel.copy()
                g = gravity.copy()
                status_vals = [calib_status[k] for k in sensors]
            
            # 1. Quaternion-Achsen
            R = quat_to_rotmat(q)
            ex, ey, ez = np.eye(3)
            ex_r = R @ ex; ey_r = R @ ey; ez_r = R @ ez
            
            b_x.set_data([0, ex_r[0]], [0, ex_r[1]])
            b_x.set_3d_properties([0, ex_r[2]])
            b_y.set_data([0, ey_r[0]], [0, ey_r[1]])
            b_y.set_3d_properties([0, ey_r[2]])
            b_z.set_data([0, ez_r[0]], [0, ez_r[1]])
            b_z.set_3d_properties([0, ez_r[2]])

            # 2. Euler als Pfeil
            euler_vec.set_data([0, e[0]], [0, e[1]])
            euler_vec.set_3d_properties([0, e[2]])

            # 3. Linear Acceleration
            lin_vec.set_data([0, la[0]], [0, la[1]])
            lin_vec.set_3d_properties([0, la[2]])

            # 4. Gravity
            grav_vec.set_data([0, g[0]], [0, g[1]])
            grav_vec.set_3d_properties([0, g[2]])

            # Kalibrierungsbalken
            for i, bar in enumerate(bars):
                bar.set_height(status_vals[i])
            
            # Grüner Rahmen bei voller Kalibrierung
            if all(v == 3 for v in status_vals):
                for spine in calib_ax.spines.values():
                    spine.set_color('green')
            else:
                for spine in calib_ax.spines.values():
                    spine.set_color('black')

            plt.tight_layout()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.5)

if __name__ == "__main__":
    main()
