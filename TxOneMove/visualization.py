import re
import time
import threading
import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ----------------- Konfiguration -----------------
PORT = "COM8"
BAUD = 115200
TIMEOUT = 1.0
# -------------------------------------------------

# Globale Variablen
gravity = np.array([0.0, 0.0, 0.0], dtype=float)
gps_pos = np.array([0.0, 0.0], dtype=float)  # [lat, lon]
gps_status = {"fix": False, "sats": 0, "hdop": 0.0, "speed": 0.0, "alt": 0.0}
calib_status = {"SYS": 0, "G": 0, "A": 0, "M": 0}
data_lock = threading.Lock()
running = True

# Regex-Patterns
gravity_pattern = re.compile(r"Gravity/BNO055_(X|Y|Z)\[raw\]:\s*([-+0-9.eE]+)")
calib_pattern = re.compile(r"Status:\s*SYS:(\d)\s*G:(\d)\s*A:(\d)\s*M:(\d)")

# GPS Patterns
gga_pattern = re.compile(r"GGA:\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)m,\s*(\d+)sats,\s*([-+0-9.eE]+),\s*FIX:(\w+)")
rmc_pattern = re.compile(r"RMC:\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)km/h,\s*(\w+)")

def serial_reader():
    global gravity, gps_pos, gps_status, calib_status, running
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    
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
            continue

        # GPS GGA
        gga_match = gga_pattern.match(line)
        if gga_match:
            lat, lon, alt, sats, hdop, fix_str = gga_match.groups()
            with data_lock:
                gps_pos = np.array([float(lat), float(lon)], dtype=float)
                gps_status["fix"] = (fix_str == "YES")
                gps_status["sats"] = int(sats)
                gps_status["hdop"] = float(hdop)
                gps_status["alt"] = float(alt)
            continue

        # GPS RMC
        rmc_match = rmc_pattern.match(line)
        if rmc_match:
            lat, lon, speed, status = rmc_match.groups()
            with data_lock:
                gps_pos = np.array([float(lat), float(lon)], dtype=float)
                gps_status["speed"] = float(speed)
                gps_status["fix"] = (status == "A")
            continue

    ser.close()

def main():
    global running

    # UART-Thread
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # Matplotlib-Setup: 1x2 Layout
    plt.ion()
    fig = plt.figure(figsize=(14, 7))
    
    # Gravity 3D (links)
    ax_grav = fig.add_subplot(121, projection="3d")
    ax_grav.set_xlim([-10, 10]); ax_grav.set_ylim([-10, 10]); ax_grav.set_zlim([-10, 10])
    ax_grav.set_xlabel("X (m/s²)"); ax_grav.set_ylabel("Y (m/s²)"); ax_grav.set_zlabel("Z (m/s²)")
    ax_grav.set_title("BNO055 Gravity")
    
    w_x, = ax_grav.plot([0, 5], [0, 0], [0, 0], '0.7', lw=1)
    w_y, = ax_grav.plot([0, 0], [0, 5], [0, 0], '0.7', lw=1)
    w_z, = ax_grav.plot([0, 0], [0, 0], [0, 5], '0.7', lw=1)
    grav_vec, = ax_grav.plot([0, 0], [0, 0], [0, 0], 'orange', lw=6)
    
    # GPS Karte + Status (rechts)
    ax_gps = fig.add_subplot(122)
    ax_gps.set_xlim(-0.01, 0.01)  # Zoom auf Position
    ax_gps.set_ylim(-0.01, 0.01)
    ax_gps.set_xlabel("Longitude")
    ax_gps.set_ylabel("Latitude")
    ax_gps.set_title("GPS Position")
    ax_gps.grid(True, alpha=0.3)
    
    # GPS Marker (roter Punkt)
    gps_marker, = ax_gps.plot([], [], 'ro', ms=12, markerfacecolor='red', markeredgecolor='darkred', markeredgewidth=2)
    gps_trail, = ax_gps.plot([], [], 'b-', alpha=0.6, lw=2)  # Bewegungspfad
    gps_pos_history = []  # Für Trail
    
    # GPS Status-Text
    gps_text = ax_gps.text(0.02, 0.98, "", transform=ax_gps.transAxes, va='top',
                          bbox=dict(boxstyle="round", facecolor='lightblue', alpha=0.8))
    
    # Kalibrierung (klein, unten)
    calib_ax = fig.add_axes([0.05, 0.02, 0.25, 0.12])
    sensors = ["SYS", "G", "A", "M"]
    colors = ["red", "orange", "yellow", "green"]
    bars = calib_ax.bar(sensors, [0,0,0,0], color=colors, alpha=0.8)
    calib_ax.set_ylim(0, 3)
    calib_ax.set_ylabel("Calib", fontsize=9)
    calib_ax.tick_params(labelsize=8)

    plt.tight_layout()

    try:
        while plt.fignum_exists(fig.number):
            # Daten kopieren
            with data_lock:
                g = gravity.copy()
                pos = gps_pos.copy()
                status = gps_status.copy()
                status_vals = [calib_status[k] for k in sensors]
            
            # Gravity aktualisieren
            grav_vec.set_data([0, g[0]], [0, g[1]])
            grav_vec.set_3d_properties([0, g[2]])
            
            # GPS Position
            if abs(pos[0]) > 1e-10 or abs(pos[1]) > 1e-10:  # Gültige Position
                gps_pos_history.append(pos.copy())
                if len(gps_pos_history) > 100:  # Max 100 Punkte Trail
                    gps_pos_history.pop(0)
                
                # Aktuelle Position
                gps_marker.set_data([pos[1]], [pos[0]])
                
                # Bewegungspfad
                if len(gps_pos_history) > 1:
                    trail_x = [p[1] for p in gps_pos_history]
                    trail_y = [p[0] for p in gps_pos_history]
                    gps_trail.set_data(trail_x, trail_y)
                else:
                    gps_trail.set_data([], [])
                
                # Zoom anpassen
                if len(gps_pos_history) > 0:
                    lats = [p[0] for p in gps_pos_history]
                    lons = [p[1] for p in gps_pos_history]
                    lat_range = 0.001 if max(lats) - min(lats) < 0.001 else max(lats) - min(lats)
                    lon_range = 0.001 if max(lons) - min(lons) < 0.001 else max(lons) - min(lons)
                    ax_gps.set_xlim(min(lons)-lon_range/2, max(lons)+lon_range/2)
                    ax_gps.set_ylim(min(lats)-lat_range/2, max(lats)+lat_range/2)
            
            # GPS Status-Text
            fix_str = "✓ FIX" if status["fix"] else "✗ NO FIX"
            gps_text.set_text(
                f'Lat: {gps_pos[0]:.6f}\n'
                f'Lon: {gps_pos[1]:.6f}\n'
                f'Sats: {status["sats"]} | HDOP: {status["hdop"]:.1f}\n'
                f'Speed: {status["speed"]:.1f} km/h\n'
                f'Alt: {status["alt"]:.1f} m\n'
                f'{fix_str}'
            )
            
            # Farbe nach Fix-Status
            marker_color = 'green' if status["fix"] else 'red'
            gps_marker.set_markerfacecolor(marker_color)
            
            # Kalibrierung
            for i, bar in enumerate(bars):
                bar.set_height(status_vals[i])
            if all(v == 3 for v in status_vals):
                calib_ax.set_facecolor('lightgreen')
            else:
                calib_ax.set_facecolor('white')

            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.5)

if __name__ == "__main__":
    main()

