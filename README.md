# rviz_rotatable_image_plugin
RViz plugin to rotate `sensor_msgs::Image` for visualisation purposes, especially so if camera is mounted at an angle.

This plugin works by taking the `sensor_msgs::Image` before it is rendered, converting it to `cv::Mat` if rotation is needed for OpenCV to perform the rotation, and converting the rotated image back to `sensor_msgs::Image` for rendering.

# Installation

1. Clone this repository to your ROS workspace
2. Build this package
    ```shell
    catkin build rviz_rotatable_image_display
    ```

# Usage

Select this plugin in RViz.

![rviz_rotatable_image_plugin in RViz](images/select_plugin.png "Plugin appears as ImageRotated under namespace rviz_rotatable_image_plugin in RViz")

# Examples

| Original | 45° Rotation | 45° Rotation No Crop |
| ----------- | ----------- | ----------- |
| ![Plugin with no rotation](images/original.png "No rotation") | ![Plugin with rotation](images/rotate_crop.png "Rotated image") | ![Plugin with rotation without crop](images/rotate_no_crop.png "Rotated image without crop option") |

# Debugging

Check if this plugin is installed properly:

```shell
rospack plugins --attrib=plugin rviz
```

You should see something like:
```shell
rviz_rotatable_image_plugin /home/user/catkin_ws/src/rviz_rotatable_image_plugin/plugin_description.xml
rviz /opt/ros/noetic/share/rviz/plugin_description.xml
```

# Acknowledgments

This plugin would not have been possible if not for the contributions from the following (and contributions therein):

1. [lucasw's answer regarding wrapping Qt headers, with code](https://answers.ros.org/question/206363/rqt-plugin-undefined-symbol/?answer=232435#post-id-232435)
2. [Rotating image without crop in C++ OpenCV](https://stackoverflow.com/a/24352524)
3. The people that contributed to OpenCV and `rviz::DisplayImage`
4. Mah weekend
