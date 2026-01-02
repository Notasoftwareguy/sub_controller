#!/usr/bin/env python3
"""
"""

# from hmac import new
from ast import Sub
from re import sub
import cv2
import pygame
import numpy as np
import sys
import time

from getcontrols import get_controls
from nonblockingstream import NonBlockingStream
from subcommunicator import SubCommunicator
from subdata import SubData

def main():
    # Configuration
    CONTROLLER_IP = '192.168.10.1'
    SUB_IP = '192.168.10.2'
    VIDEO_PORT = 5000
    SURFACE_DATA_PORT = 6000
    SUB_DATA_PORT = 7000
    WINDOW_WIDTH = 1280
    WINDOW_HEIGHT = 720
    FPS = 30
    CONTROL_UPDATE_RATE_HZ = 50
    
    
    gst_pipeline = (
        f"udpsrc port={VIDEO_PORT} ! "
        "application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! "
        "rtph264depay ! "
        "avdec_h264 ! "
        "videoconvert ! "
        "appsink"
    )

    # Configure video stream
    cap = NonBlockingStream(gst_pipeline)
    cap.start()
    
    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("GStreamer Video Stream (OpenCV)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 36)  # None = default font, 36 = size
    print("Press Ctrl+C or close the window to exit.\n")

    # Sub communication setup
    received_data = SubData()
    surface_data_server = SubCommunicator(host=CONTROLLER_IP, port=SURFACE_DATA_PORT)
    surface_data_server.receive_callback = received_data.add_data
    surface_data_server.start_server()
    sub_data_client = SubCommunicator(host=SUB_IP, port=SUB_DATA_PORT)

    # Final variable setup
    last_frame_time = time.time()
    elapsed_time = 0
    frame = None
    original_frame = None
    current_window_width = WINDOW_WIDTH
    current_window_height = WINDOW_HEIGHT
    reported_failure = True
    last_control_transmit_time = time.time()

    # Main loop
    try:
        while True:
            # Clear the screen each frame
            screen.fill((0, 0, 0))
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                elif event.type == pygame.VIDEORESIZE:
                    current_window_width, current_window_height = event.size

            ### Camera View ###
            # Get latest camera frame and update screen
            ret, new_frame = cap.read()
            if ret and new_frame is not None:
                original_frame = new_frame
                frame = cv2.cvtColor(original_frame, cv2.COLOR_BGR2RGB)
                last_frame_time = time.time()

            if frame is not None:
                # Resize frame to fit window while maintaining aspect ratio
                frame_height, frame_width = frame.shape[:2]
                aspect_ratio = frame_width / frame_height
                
                # Calculate dimensions to fit in window
                new_width = current_window_width
                new_height = int(current_window_width / aspect_ratio)
                
                # If height exceeds window height, scale based on height instead
                if new_height > current_window_height:
                    new_height = current_window_height
                    new_width = int(current_window_height * aspect_ratio)
                
                # Resize frame
                resized_frame = cv2.resize(frame, (new_width, new_height))
                
                # Convert to pygame surface and blit centered
                frame_surface = pygame.surfarray.make_surface(np.flip(np.rot90(resized_frame), 0))
                x_offset = (current_window_width - new_width) // 2
                y_offset = (current_window_height - new_height) // 2
                screen.blit(frame_surface, (x_offset, y_offset))
                
            # Display time since last frame in bottom right
            elapsed_time = time.time() - last_frame_time
            time_text = font.render(f"{elapsed_time:.1f}s", True, (255, 255, 255))
            text_rect = time_text.get_rect(bottomright=(current_window_width - 10, current_window_height - 10))
            screen.blit(time_text, text_rect)
            
            ### Controls ###
            if sub_data_client.connection is None:
                success = sub_data_client.connect()

                if not reported_failure and not success:
                    print("Lost data connection to sub!")
                    reported_failure = True
                if success:
                    print("Reconnected to sub data.")
                    reported_failure = False

            if sub_data_client.connection is not None:
                controls = get_controls()
                if not sub_data_client.send('STICKS ' + ' '.join(map(str, controls))):
                    sub_data_client.close()
                

            ### Menu/Telemetry ###




            pygame.display.flip()
            clock.tick(FPS)
            

        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        surface_data_server.close()
        sub_data_client.close()
        cap.stop()
        pygame.quit()
        print("Done!")

if __name__ == '__main__':
    main()
