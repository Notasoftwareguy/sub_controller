import cv2
import threading
import time

class NonBlockingStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        if self.started:
            return None
        self.started = True
        self.updated = False
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True # Thread dies when main script exits
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
                self.updated = True

    def read(self):
        with self.read_lock:
            if self.updated:
                frame = self.frame.copy() if self.grabbed else None
                grabbed = self.grabbed
                self.updated = False
            else:
                frame = None
                grabbed = False

        return grabbed, frame

    def stop(self):
        self.started = False
        self.thread.join()

# if __name__ == "__main__":
#     # --- Usage ---
#     pipeline = "udpsrc port=5000 ! application/x-rtp,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true"

#     stream = NonBlockingStream(pipeline).start()

#     while True:
#         ret, frame = stream.read()
        
#         if not ret or frame is None:
#             print("Waiting for frame...")
#             time.sleep(0.1)
#             continue

#         cv2.imshow("Non-Blocking Stream", frame)
        
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     stream.stop()
#     cv2.destroyAllWindows()