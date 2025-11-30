import socket
import json
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

HOST = "127.0.0.1"
PORT = 5555

# --- Shared state (thread-safe append, read-only from plot thread) ---

MAX_POINTS = 300

time_axis = deque(maxlen=MAX_POINTS)

lidar_range = deque(maxlen=MAX_POINTS)
azimuth = deque(maxlen=MAX_POINTS)
elevation = deque(maxlen=MAX_POINTS)
stability = deque(maxlen=MAX_POINTS)
tracker_conf = deque(maxlen=MAX_POINTS)
engage_state = deque(maxlen=MAX_POINTS)

lock = threading.Lock()

def reader_thread():
    """
    Reads JSON lines from the TCP connection and updates shared buffers.
    """
    global time_axis

    while True:
        try:
            print(f"[PlotServer] Listening on {HOST}:{PORT}")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((HOST, PORT))
            sock.listen(1)

            conn, addr = sock.accept()
            print(f"[PlotServer] Connected from {addr}")

            with conn, conn.makefile("r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        print(f"[PlotServer] Invalid JSON: {line}")
                        continue

                    kind = msg.get("kind", "")

                    with lock:
                        ts = time.time()
                        time_axis.append(ts)

                        if kind == "sensor":
                            lidar_range.append(float(msg.get("range", 0.0)))

                        elif kind == "solution":
                            azimuth.append(float(msg.get("az", 0.0)))
                            elevation.append(float(msg.get("el", 0.0)))
                            stability.append(float(msg.get("st", 0.0)))

                        elif kind == "tracker":
                            tracker_conf.append(float(msg.get("conf", 0.0)))

                        elif kind == "engage":
                            engage_state.append(int(msg.get("state", 0)))

                        else:
                            print(f"[PlotServer] Unknown kind: {kind}")

            print("[PlotServer] Connection closed, waiting for new client...")
            sock.close()

        except Exception as e:
            print(f"[PlotServer] Error: {e}")
            time.sleep(1.0)


def setup_figure():
    """
    Creates the 4-panel matplotlib layout.
    """
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    ax_lidar, ax_sol, ax_conf, ax_eng = axes

    # Lidar range
    ax_lidar.set_ylabel("LIDAR range")
    line_lidar, = ax_lidar.plot([], [], label="range")
    ax_lidar.legend(loc="upper right")
    ax_lidar.grid(True)

    # Solution (azimuth / elevation / stability)
    ax_sol.set_ylabel("Solution")
    line_az, = ax_sol.plot([], [], label="azimuth")
    line_el, = ax_sol.plot([], [], label="elevation")
    line_st, = ax_sol.plot([], [], label="stability")
    ax_sol.legend(loc="upper right")
    ax_sol.grid(True)

    # Tracker confidence
    ax_conf.set_ylabel("Confidence")
    line_conf, = ax_conf.plot([], [], label="tracker conf")
    ax_conf.set_ylim(0.0, 1.05)
    ax_conf.legend(loc="upper right")
    ax_conf.grid(True)

    # Engagement state (state machine output)
    ax_eng.set_ylabel("Engage state")
    ax_eng.set_xlabel("time (s)")
    line_eng, = ax_eng.plot([], [], drawstyle="steps-post", label="state")
    ax_eng.set_yticks(range(0, 8))
    ax_eng.grid(True)

    fig.tight_layout()
    return fig, (ax_lidar, ax_sol, ax_conf, ax_eng), (
        line_lidar, line_az, line_el, line_st, line_conf, line_eng
    )


def animate(_frame, axes, lines):
    """
    Updates plot lines every ~100ms using buffered data.
    """
    (ax_lidar, ax_sol, ax_conf, ax_eng) = axes
    (line_lidar, line_az, line_el, line_st, line_conf, line_eng) = lines

    with lock:
        if not time_axis:
            return lines

        t0 = time_axis[0]
        xs = [t - t0 for t in time_axis]

        # Lidar
        if lidar_range:
            line_lidar.set_data(xs[-len(lidar_range):], list(lidar_range))
            ax_lidar.relim()
            ax_lidar.autoscale_view()

        # Solution
        if azimuth:
            line_az.set_data(xs[-len(azimuth):], list(azimuth))
        if elevation:
            line_el.set_data(xs[-len(elevation):], list(elevation))
        if stability:
            line_st.set_data(xs[-len(stability):], list(stability))
        ax_sol.relim()
        ax_sol.autoscale_view()

        # Confidence
        if tracker_conf:
            line_conf.set_data(xs[-len(tracker_conf):], list(tracker_conf))
            ax_conf.relim()
            ax_conf.autoscale_view()

        # State machine output
        if engage_state:
            line_eng.set_data(xs[-len(engage_state):], list(engage_state))
            ax_eng.relim()
            ax_eng.autoscale_view()

    return lines


def main():
    # Spawn reader thread
    t = threading.Thread(target=reader_thread, daemon=True)
    t.start()

    # Prepare figure
    fig, axes, lines = setup_figure()

    ani = animation.FuncAnimation(
        fig,
        func=animate,
        fargs=(axes, lines),
        interval=100,
        blit=False
    )

    print("[PlotServer] Starting GUI loop...")
    plt.show()


if __name__ == "__main__":
    main()