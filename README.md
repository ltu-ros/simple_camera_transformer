# `simple_camera_transformer`

This package takes an image input, applys dynamic transformations such as brightness and contrast, and re-publishes the transformed image. 

To launch:

```
roslaunch simple_camera_transformer transformer.launch
```

Specify a camera input to transform in the launch file:

```xml
<node name="cam_transformer" pkg="simple_camera_transformer" type="cam_transformer" respawn="true" respawn_delay="10" output="screen">
    <param name="pub_topic"   type="string" value="/my_camera/cam_pub/image_transformed" />
    <param name="sub_topic"   type="string" value="/my_camera/cam_pub/image_raw" />
</node>

```

## Configuration

This package uses [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure). Launch `rqt` and open Plugins > Configuration > Dynamic Reconfigure.
