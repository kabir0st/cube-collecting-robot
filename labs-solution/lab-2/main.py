import socket
import struct
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

HOST = "127.0.0.1"
PORT = 9890
HISTORY = 4
LIM_MM = 6000


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def reader_loop(conn, state, lock):
    try:
        while True:
            hdr = recv_exact(conn, 4)
            if hdr is None:
                break
            (count,) = struct.unpack("<I", hdr)
            if count == 0:
                continue
            payload = recv_exact(conn, count * 8)
            if payload is None:
                break
            arr = np.frombuffer(payload, dtype="<i4").reshape(-1, 2)
            angle_deg = arr[:, 0].astype(np.float32) / 100.0
            dist_mm = arr[:, 1].astype(np.float32)
            theta = np.deg2rad(angle_deg)
            x = dist_mm * np.cos(theta)
            y = dist_mm * np.sin(theta)
            xy = np.column_stack((x, y))
            with lock:
                state["batches"].append(xy)
                state["dirty"] = True
    except (ConnectionResetError, OSError):
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"listening on {HOST}:{PORT}")

    conn, addr = srv.accept()
    print(f"client connected: {addr}")

    state = {"batches": deque(maxlen=HISTORY), "dirty": False}
    lock = threading.Lock()

    t = threading.Thread(target=reader_loop, args=(conn, state, lock), daemon=True)
    t.start()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_xlim(-LIM_MM, LIM_MM)
    ax.set_ylim(-LIM_MM, LIM_MM)
    ax.grid(True, alpha=0.3)
    ax.set_title("Lidar Map (mm)")
    ax.plot(0, 0, marker="+", color="red", markersize=12)
    scat = ax.scatter([], [], s=4, c="C0")

    def update(_frame):
        with lock:
            if not state["dirty"]:
                return (scat,)
            state["dirty"] = False
            if not state["batches"]:
                return (scat,)
            xy = np.vstack(state["batches"])
        scat.set_offsets(xy)
        return (scat,)

    _anim = FuncAnimation(fig, update, interval=1000 / 24, blit=True, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass
        srv.close()


if __name__ == "__main__":
    main()
