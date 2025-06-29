import socket, time
import cv2, numpy as np
import pyvirtualcam           # NEW
from pyvirtualcam import PixelFormat

# --- CONFIG --------------------------
UDP_IP   = "0.0.0.0"
UDP_PORT = 12345
MAX_PKT  = 65507               # máx. UDP
# -------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(5.0)
print(f"Listening on UDP {UDP_PORT}…")

buffer      = b""
frame_count = 0
fps_print   = 0
t0          = time.time()


WIDTH, HEIGHT, FPS = 1024, 768, 30
with pyvirtualcam.Camera(width=WIDTH,
                         height=HEIGHT,
                         fps=FPS,
                         fmt=PixelFormat.BGR) as cam:
    print(f"Virtual cam: {cam.device}")

    while True:
        try:
            data, _ = sock.recvfrom(MAX_PKT)

            if data.startswith(b'\xff\xd8'):   # SOI
                buffer = b""
            buffer += data

            if data.endswith(b'\xff\xd9'):     # EOI
                jpg   = np.frombuffer(buffer, np.uint8)
                frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)  # BGR
                if frame is None:
                    continue

                # resize si el frame no coincide con la cam virtual
                if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
                    frame = cv2.resize(frame, (WIDTH, HEIGHT),
                                       interpolation=cv2.INTER_AREA)

                # --- FPS consola ---
                frame_count += 1
                if time.time() - t0 >= 1.0:
                    fps_print = frame_count
                    frame_count = 0
                    t0 = time.time()
                    print(f"FPS recv: {fps_print}")

                # --- enviar a la webcam virtual --------------
                cam.send(frame)               # BGR ⇒ PixelFormat.BGR
                cam.sleep_until_next_frame()  # temporiza al objetivo

        except socket.timeout:
            print("UDP timeout")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("Error:", e)

sock.close()
