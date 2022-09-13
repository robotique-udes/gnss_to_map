# gnss_to_map
Converts GNSS coordinates (sensor_msgs/NavSatFix) into map frame coordinates in meters (geometry_msgs/PoseWithCovarianceStamped). This makes it possible to use GNSS data with the ROS navigation stack.

## Subscribed topics
- fix (sensor_msgs/NavSatFix): GNSS coordinates data.
- local_xy_origin (geometry_msgs/PoseStamped): Latitude and longitude coordinate of the origin of the map frame. This message is fetched only once when this node is launched.

## Published topics
- gnss_map_pose (geometry_msgs/PoseWithCovarianceStamped): GNSS coordinates converted to map coordinates in meters.

## Initializing map origin
To publish the `local_xy_origin` topic that is needed by this node, you can use  the `initialize_origin.py` node from the `swri_transform_util` package.

 Example launch file:
 ``` xml
 <node pkg="swri_transform_util" type="initialize_origin.py" name="initialize_origin" >
    <param name="local_xy_frame" value="/map"/>
    <param name="local_xy_origin" value="auto"/>  <!-- Change this to the name of the wanted location in the list or auto for the first GNSS coordinate -->
    <rosparam param="local_xy_origins">
      [{ name: MDRS,
         latitude: 38.406333,
         longitude: -110.791962,
         altitude: 1372.0,
         heading: 0.0},
         
         { name: UdeS,
          latitude: 45.378395,
          longitude: -71.926644,
          altitude: 263.0,
          heading: 0.0}]
    </rosparam>
  </node> 
 ```