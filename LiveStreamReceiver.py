import socket
import cv2
import time
import io
import numpy as np


def liveStream():
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('192.168.4.1', 8888))
            buffer = bytearray()
            while True:
                chunk = s.recv(8192)
                if chunk:
                    buffer.extend(chunk)
                else: break


                # Find JPEG start and end
                while True:
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9', start)
                    if start != -1 and end != -1:
                        frame = buffer[start:end+2]
                        buffer = buffer[end+2:]

                        # Decode and display
                        img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            cv2.imshow("Live Stream", img)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                cv2.destroyAllWindows()
                                return
                        else:
                            print("Failed to decode")
                    else:
                        
                        time.sleep(.1)
                        break

if __name__ == "__main__":
    liveStream()
