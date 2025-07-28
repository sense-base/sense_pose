import mediapipe as mp
import cv2
import pyzed.sl as sl
import time
import numpy as np

def standalone_webcam_pose_detection():
    '''
    Function that uses MediaPipe Pose Landmarker to estimate the position of key body locations 
    on a livestream of input images from the webcam

    For MediaPipe Pose Landmarker see https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
    '''

    # Webcam video feed
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    # Import mediapipe model and drawing utilities
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    prev_time = time.time() 

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

        while cap.isOpened():
            # Capture video frame
            ret, frame = cap.read()
            if not ret:
                    print("Error: Failed to capture frame.")
                    break
            

            # Calculate FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Print FPS to the console
            print(f"FPS: {fps:.2f}")

            # Recolor image to RGB
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            
            # Pose detection
            results = pose.process(image)

            # Color back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract landmarks
            #landmarks = results.pose_landmarks.landmark

            # Render detections
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Display webcam feed with detected landmarks/skeleton - press 'Q' to exit
            cv2.imshow("Mediapipe Feed", image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()



def standalone_zedcam_pose_detection():
    '''
    Function that uses MediaPipe Pose Landmarker to estimate the position of key body locations 
    on a livestream of input images from the ZED camera.

    For MediaPipe Pose Landmarker see https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
    '''

    # Initialize ZED camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Enable depth sensing
    init_params.coordinate_units = sl.UNIT.METER  # Use meters for 3D coordinates

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error: {status}")
        exit()

    # Get camera intrinsics
    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
    fx = calibration_params.fx  # Focal length in x
    fy = calibration_params.fy  # Focal length in y
    cx = calibration_params.cx  # Principal point x
    cy = calibration_params.cy  # Principal point y

    # Create runtime parameters and Mat objects to hold the image and depth info
    runtime_params = sl.RuntimeParameters()
    zed_image = sl.Mat()
    zed_depth = sl.Mat()

    # Import mediapipe model and drawing utilities
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:
            # Capture image from ZED camera
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve the image from left camera
                zed.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame = zed_image.get_data()  # Get image as numpy array

                # Retrieve depth map
                zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH)

                # Convert ZED image to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Pose detection
                results = pose.process(image)

                # Color back to BGR for OpenCV display
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Extract landmarks and compute 3D coordinates
                if results.pose_landmarks:
                    landmarks = results.pose_landmarks.landmark
                    image_h, image_w = frame.shape[:2]
                    for idx, landmark in enumerate(landmarks):
                        # Get 2D coordinates (normalized to [0,1])
                        x, y = landmark.x, landmark.y
                        # Convert to pixel coordinates
                        pixel_x = int(x * image_w)
                        pixel_y = int(y * image_h)
                        # Ensure coordinates are within image bounds
                        if 0 <= pixel_x < image_w and 0 <= pixel_y < image_h:
                            # Get depth value at (pixel_x, pixel_y)
                            depth_value = zed_depth.get_value(pixel_x, pixel_y)[1]
                            if np.isfinite(depth_value):  # Check for valid depth
                                # Compute 3D coordinates
                                Z = depth_value
                                X = (pixel_x - cx) * Z / fx
                                Y = (pixel_y - cy) * Z / fy
                                if idx == 13:
                                    print(f"Left elbow: 3D (X, Y, Z) = ({X:.3f}, {Y:.3f}, {Z:.3f}) m")
                            elif idx == 13:
                                print(f"Left elbow: Invalid depth at ({pixel_x}, {pixel_y})")
                        elif idx == 13:
                            print(f"Left elbow: Out of bounds at ({pixel_x}, {pixel_y})")

                # Render detections
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                # Display ZED feed with detected landmarks/skeleton - press 'Q' to exit
                cv2.imshow("Mediapipe ZED Feed", image)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        zed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    #  standalone_webcam_pose_detection()
    standalone_zedcam_pose_detection()