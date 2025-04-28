import mediapipe as mp
import cv2
import numpy as np
import pyzed.sl as sl
import time

def standalone_webcam_pose_detection():
    '''
    Function that uses MediaPipe Pose Landmarker to estimate the position of key body locations 
    on a livestream of input images from the webcam

    For MediaPipe Pose Landmarker see https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
    '''

    # Webcam video feed
    cap = cv2.VideoCapture(0)
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
            try:
                 landmarks = results.pose_landmarks.landmark
                 #print(landmarks)
            except:
                 pass

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

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error: {status}")
        exit()

    # Create runtime parameters and Mat to hold the image
    runtime_params = sl.RuntimeParameters()
    zed_image = sl.Mat()

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

                # Convert ZED image to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Pose detection
                results = pose.process(image)

                # Color back to BGR for OpenCV display
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Extract landmarks
                try:
                    landmarks = results.pose_landmarks.landmark
                    # print(landmarks) 
                except:
                    pass

                # Render detections
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                # Display ZED feed with detected landmarks/skeleton - press 'Q' to exit
                cv2.imshow("Mediapipe ZED Feed", image)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        zed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
     standalone_webcam_pose_detection()
     #standalone_zedcam_pose_detection()