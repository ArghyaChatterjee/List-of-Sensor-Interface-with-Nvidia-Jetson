You need to have ROS installed previously. Clone the usb_cam package into the 'src' folder of 'catkin_ws':
```
cd ~/catkin_ws/src/
git clone https://github.com/bosch-ros-pkg/usb_cam.git
```
Build the workspace using catkin_make:
```
cd ..
catkin_make
```
After building the package, install the v4l-util Ubuntu package. It is a collection of
command-line V4L (Video for Linux) utilities used by the usb_cam package:
```
sudo apt-get install v4l-utils
```
Now, copy the camera_info folder (inside ~/catkin_ws/src/usb-cam) to .ros folder inside your 'Home' directory. You can see by default a 'head_camera.yaml' file (camera calibration file) inside camera_info folder. This is needed when using usb_cam package. 'Calibration file' is camera brand as well as resolution specific. The calibration files for different camera brand & resolution has been included inside 'usb_cam' package of this repo. You can just rename anyfile to 'head_camera.yaml' & paste it inside ~/.ros/camera_info directory.  
