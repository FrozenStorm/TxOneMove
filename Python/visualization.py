import re
import time
import threading
import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import contextily as ctx
import warnings
warnings.filterwarnings("ignore")

# ----------------- Konfiguration -----------------
PORT = "COM8"
BAUD = 115200
TIMEOUT = 1.0
SWITZERLAND_BBOX = [5.95, 45.75, 10.6, 47.85]  # Lon_min, Lat_min, Lon_max, Lat_max
# -------------------------------------------------

# Globale Variablen
gravity = np.array([0.0, 0.0, 0.0], dtype=float)
gps_pos = np.array([46.8182, 8.2275], dtype=float)  # Schweiz Center Start
gps_status = {"fix": False, "sats": 0, "hdop": 99.0, "speed": 0.0, "alt": 0.0}
gps_history = []
calib_status = {"SYS": 0, "G": 0, "A": 0, "M": 0}
data_lock = threading.Lock()
running = True

# ✅ NEUE Regex-Patterns für TinyGPS++
gravity_pattern = re.compile(r"Gravity/BNO055_(X|Y|Z)\[raw\]:\s*([-+0-9.eE]+)")
calib_pattern = re.compile(r"Status:\s*SYS:(\d+)\s*G:(\d+)\s*A:(\d+)\s*M:(\d+)")

# TinyGPS++ Patterns
tinygps_pattern = re.compile(r"GPS/TinyGPS:\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\s*\|\s*Speed:\s*([-+0-9.eE]+)\s*km/h\s*\|\s*Alt:\s*([-+0-9.eE]+)\s*m")
gps_stats_pattern = re.compile(r"GPS/Sats:\s*(\d+)\s*\|\s*HDOP:\s*([-+0-9.eE]+)\s*\|\s*FixAge:\s*(\d+)")

def serial_reader():
    global gravity, gps_pos, gps_status, gps_history, calib_status, running
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    grav_current = {"X": None, "Y": None, "Z": None}

    while running:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        # Kalibrierung
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

        # ✅ TinyGPS++ Position + Speed/Alt
        tinygps_match = tinygps_pattern.match(line)
        if tinygps_match:
            lat, lon, speed, alt = tinygps_match.groups()
            with data_lock:
                gps_pos = np.array([float(lat), float(lon)], dtype=float)
                gps_status["speed"] = float(speed)
                gps_status["alt"] = float(alt)
                gps_status["fix"] = True  # TinyGPS gibt nur gültige Positionen
                
                # History (alle 10m)
                if (len(gps_history) == 0 or 
                    np.linalg.norm(gps_pos - gps_history[-1]) > 0.0001):  # ~10m
                    gps_history.append(gps_pos.copy())
                    if len(gps_history) > 500:
                        gps_history.pop(0)
            continue

        # ✅ TinyGPS++ Sats/HDOP
        gps_stats_match = gps_stats_pattern.match(line)
        if gps_stats_match:
            sats, hdop, fix_age = gps_stats_match.groups()
            with data_lock:
                gps_status["sats"] = int(sats)
                gps_status["hdop"] = float(hdop)
                gps_status["fix_age"] = int(fix_age)
            continue

    ser.close()

def main():
    global running

    # UART-Thread starten
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # Matplotlib Dashboard 2x2
    plt.ion()
    fig = plt.figure(figsize=(16, 10))

    # 1. Gravity 3D (oben links)
    ax_grav = fig.add_subplot(221, projection="3d")
    ax_grav.set_xlim([-10, 10]); ax_grav.set_ylim([-10, 10]); ax_grav.set_zlim([-10, 10])
    ax_grav.set_title("🧭 BNO055 Gravity Vektor")
    ax_grav.set_xlabel("X (m/s²)"); ax_grav.set_ylabel("Y (m/s²)"); ax_grav.set_zlabel("Z (m/s²)")
    
    w_x, = ax_grav.plot([0, 5], [0, 0], [0, 0], '0.7', lw=1)
    w_y, = ax_grav.plot([0, 0], [0, 5], [0, 0], '0.7', lw=1)
    w_z, = ax_grav.plot([0, 0], [0, 0], [0, 5], '0.7', lw=1)
    grav_vec, = ax_grav.plot([0, 0], [0, 0], [0, 0], 'orange', lw=6)

    # 2. Schweiz Karte mit GPS (oben rechts)
    ax_map = fig.add_subplot(222)
    ax_map.set_title("🇨🇭 Schweiz - TinyGPS++ Live Tracking")
    ax_map.set_aspect('equal')
    
    print("📍 Lade Schweiz-Karte...")
    ax_map.set_xlim(SWITZERLAND_BBOX[0], SWITZERLAND_BBOX[2])
    ax_map.set_ylim(SWITZERLAND_BBOX[1], SWITZERLAND_BBOX[3])
    ax_map.invert_yaxis()
    
    ax_map.set_xlabel("Longitude")
    ax_map.set_ylabel("Latitude")
    ax_map.grid(True, alpha=0.3)
    
    # GPS Elemente
    gps_marker, = ax_map.plot([], [], 'o', ms=15, mec='darkred', mew=3, 
                              markerfacecolor='limegreen', label="GPS Live (TinyGPS++)")
    gps_trail, = ax_map.plot([], [], '-', color='blue', alpha=0.7, lw=3, label="Trail")
    
    # Schweiz Rahmen
    schweiz_border = plt.Rectangle((SWITZERLAND_BBOX[0], SWITZERLAND_BBOX[1]), 
                                   SWITZERLAND_BBOX[2]-SWITZERLAND_BBOX[0], 
                                   SWITZERLAND_BBOX[3]-SWITZERLAND_BBOX[1],
                                   facecolor='none', edgecolor='black', lw=2, alpha=0.8)
    ax_map.add_patch(schweiz_border)
    ax_map.legend(loc='upper left')

    # 3. Kalibrierung (unten links)
    ax_calib = fig.add_subplot(223)
    sensors = ["SYS", "G", "A", "M"]
    colors = ["red", "orange", "yellow", "green"]
    bars = ax_calib.bar(sensors, [0,0,0,0], color=colors, alpha=0.8)
    ax_calib.set_ylim(0, 3.2)
    ax_calib.set_title("BNO055 Kalibrierung")
    ax_calib.set_ylabel("Status (0-3)")

    # 4. GPS Status (unten rechts)
    ax_gps_status = fig.add_subplot(224)
    ax_gps_status.axis('off')
    gps_text = ax_gps_status.text(0.05, 0.95, "", va='top', fontsize=11, 
                                  bbox=dict(boxstyle="round", facecolor='lightblue', alpha=0.9))

    plt.tight_layout()

    try:
        while plt.fignum_exists(fig.number):
            with data_lock:
                g = gravity.copy()
                pos = gps_pos.copy()
                status = gps_status.copy()
                calib_vals = [calib_status[k] for k in sensors]
                hist_copy = gps_history.copy()

            # Gravity
            grav_vec.set_data([0, g[0]], [0, g[1]])
            grav_vec.set_3d_properties([0, g[2]])

            # GPS Karte
            if len(hist_copy) > 0 and np.any(pos != 0):
                # Live Marker (grün bei gültiger Position)
                marker_color = 'limegreen' if status["fix"] else 'red'
                gps_marker.set_data([pos[1]], [pos[0]])
                gps_marker.set_markerfacecolor(marker_color)
                
                # Trail (letzte 100 Punkte)
                if len(hist_copy) > 1:
                    trail_lons = [p[1] for p in hist_copy[-100:]]
                    trail_lats = [p[0] for p in hist_copy[-100:]]
                    gps_trail.set_data(trail_lons, trail_lats)
                else:
                    gps_trail.set_data([], [])
                
                # Auto-Zoom bei Bewegung
                if len(hist_copy) > 5:
                    lats = np.array([p[0] for p in hist_copy[-20:]])
                    lons = np.array([p[1] for p in hist_copy[-20:]])
                    lat_pad = 0.002; lon_pad = 0.003
                    ax_map.set_ylim(np.min(lats)-lat_pad, np.max(lats)+lat_pad)
                    ax_map.set_xlim(np.min(lons)-lon_pad, np.max(lons)+lon_pad)

            # GPS Status Text (TinyGPS++ Format)
            fix_icon = "✅ TinyGPS++ FIX" if status["fix"] else "❌ Waiting..."
            dist_traveled = 0
            if len(hist_copy) > 1:
                diffs = np.diff(np.array(hist_copy), axis=0)
                dist_traveled = np.sum(np.sqrt(np.sum(diffs**2, axis=1))) * 111000  # ~m
            
            gps_text.set_text(
                f'{fix_icon}\n\n'
                f'📍 Live Position:\n'
                f'  Lat: {pos[0]:8.6f}°\n'
                f'  Lon: {pos[1]:8.6f}°\n\n'
                f'📡 Sats: {status["sats"]} | HDOP: {status["hdop"]:.1f}\n'
                f'🚀 Speed: {status["speed"]:.1f} km/h\n'
                f'⛰️  Alt: {status["alt"]:.0f} m\n\n'
                f'🗺️  Track: {len(gps_history)} Pkt\n'
                f'   Dist: {dist_traveled:.0f} m\n\n'
                f'💡 Google Maps:\n'
                f'   {pos[0]:.6f}, {pos[1]:.6f}'
            )

            # Kalibrierung
            for i, bar in enumerate(bars):
                bar.set_height(calib_vals[i])
            if all(v == 3 for v in calib_vals):
                ax_calib.set_facecolor('lightgreen')
                ax_calib.set_title("✅ BNO055 KALIBRIERT", color='darkgreen')
            else:
                ax_calib.set_facecolor('white')
                ax_calib.set_title("BNO055 Kalibrierung", color='black')

            # Karten-Hintergrund (einmalig)
            try:
                if not hasattr(main, 'map_loaded') or not main.map_loaded:
                    ctx.add_basemap(ax_map, crs='EPSG:4326', source=ctx.providers.OpenStreetMap.Mapnik)
                    main.map_loaded = True
            except:
                pass

            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.5)
        plt.savefig('gps_gravity_tinygps.png', dpi=150, bbox_inches='tight')
        print("✅ Screenshot: gps_gravity_tinygps.png")

if __name__ == "__main__":
    main()
