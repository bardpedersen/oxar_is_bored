import cv2
import pyrealsense2 as rs
import numpy as np

# Create a RealSense pipeline
pipeline = rs.pipeline()

# Create a configuration object and enable the color and depth streams
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get the color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert the color frame to a numpy array
        color_image = np.array(color_frame.get_data())

        # Convert the depth frame to a numpy array
        depth_image = np.array(depth_frame.get_data())

        # Display the color and depth images
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image', depth_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()

# Close all OpenCV windows
cv2.destroyAllWindows()
