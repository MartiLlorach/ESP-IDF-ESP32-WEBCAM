import socket
import cv2
import numpy as np
import time

# === CONFIG ===
UDP_IP = "0.0.0.0"         # Listen on all interfaces
UDP_PORT = 12345           # Match the port your ESP32 sends to
MAX_PACKET_SIZE = 65507    # Max size of a UDP packet

# === SETUP SOCKET ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

print(f"Listening for JPEG frames on UDP {UDP_PORT}...")

# === DISPLAY LOOP ===
buffer = b""
frame_count = 0
fps = 0
last_fps_time = time.time()

while True:
    try:
        data, addr = sock.recvfrom(MAX_PACKET_SIZE)

        # Optional: Use a start-of-frame marker
        if data.startswith(b'\xff\xd8'):  # JPEG SOI marker
            buffer = b""

        buffer += data

        if data.endswith(b'\xff\xd9'):  # JPEG EOI marker
            # Decode JPEG buffer
            jpg = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            if frame is not None:
                # === FPS COUNTER ===
                frame_count += 1
                now = time.time()
                if now - last_fps_time >= 1.0:
                    fps = frame_count
                    frame_count = 0
                    last_fps_time = now
                    print(f"FPS: {fps}")

                # Optional: Show FPS on image
                cv2.putText(frame, f"FPS: {fps}", (10, 30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow("UDP Stream", frame)

            key = cv2.waitKey(1)
            if key == 27:  # ESC to exit
                break

    except socket.timeout:
        print("Timeout waiting for UDP packets.")
    except Exception as e:
        print("Error:", e)

sock.close()
cv2.destroyAllWindows()
