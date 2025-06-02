# sense_pose

This is a respository containing documentation and code for a ROS2 pose estimation package.
Note - The file `mpipe_standalone_pose_detection.py` is used as a standalone script to test the pose detection model outside the ROS2 package.

## Setup
If running the code for the first time on your device, open a command line console and enter the following command to create a configuration file:

`sudo nano /etc/sysctl.d/10-cyclone-max.conf`

Paste the following into the file:

```
# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB

# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB
```

Save the file and reboot.

## Run
Build the workspace:
`colcon build --symlink-install`

Once that finishes, source the installation files:
`source install/setup.bash`

Then run the demo:
`ros2 launch mpipe mpipe_webcam_launch.py`
or
`ros2 launch mpipe mpipe_zedcam_launch.py`
depending on whether you want to use a webcam or ZED camera

### View
To view the output, open another terminal and run:
`rqt`

Use the plugins file to visualise Images, there should be a `/pose_skeleton` topic that can visualise the estimated pose.

## Troubleshooting
### `/usr/lib64/libEGL_nvidia.so.***.**.**: No such file or directory`

If the container fails to build and the log file containes the above error, this is due to a recent update on the host. The podman container searches for the GPU runtime libraries through a predefined configuration file (nvidia.yaml) that locates the libraries on the host. When the drivers are updated, those libraries don't exist anymore and therefore the container can't find them to load the cuda runtime. 

To fix, run `sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml` and restart VSCode.

