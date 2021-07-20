# darpa_reporting

A ROS node to transform and publish the map point cloud and the robots poses into the `darpa` frame so it can be send to the DARPA server with the [`mapping_relay`](https://bitbucket.org/subtchallenge/test_mapping_server/src/master/mapping_relay/) node.

## Node parameters

|        Name         |                         Description                         | Possible values  | Default Value |
| :-----------------: | :---------------------------------------------------------: | :--------------: | :-----------: |
|    robot_frames     |              The list for the robots frame id.              | A list of string | "[base_link]" |
|     darpa_frame     |                 The DARPA reference frame.                  |    Any string    |    "darpa"    |
|      map_frame      |                  The map reference frame.                   |    Any string    |     "map"     |
|      map_topic      |            The topic to subscribed for the map.             |    Any string    |     "map"     |
|  map_publish_topic  |          The topic on which the map is published.           |    Any string    |  "map_relay"  |
| poses_publish_topic |         The topic on which the poses is published.          |    Any string    | "poses_relay" |
|  map_publish_rate   |  The rate at which the map is published (must be < 1 Hz).   |      [0,1]       |      0.1      |
| poses_publish_rate  | The rate at which the poses are published (must be < 1 Hz). |      [0,1]       |      0.1      |
| transform_dimension |        Homogeneous dimension of the transformation.         |      {3,4}       |       4       |

## Testing

To test this package, clone [this repository](https://bitbucket.org/subtchallenge/test_mapping_server/src/master/) into your catkin workspace then follow the instructions for the [`mapping_server`](https://bitbucket.org/subtchallenge/test_mapping_server/src/master/mapping_server/) and [`mapping_relay`](https://bitbucket.org/subtchallenge/test_mapping_server/src/master/mapping_relay/) packages.

The number of robots in the `robot_frames` and `robot_names` lists must be the same in the `mapping_relay.launch` file and the order in both list must also match, e.g. `"robot_frames: [base_link1, base_link2, ...]", robot_names: "[robot1, robot2, ...]"`.

Since this package is only relaying the map and the pose of the robots, everything related to occupancy grid and markers subscription can be commented out in the `mapping_relay.py` and the `mapping_relay.launch` files. The `rosbag` recording in the `mapping_server.launch` file can also be commented out if not desired.

Then, run the following commands in separated terminals:

1. `roscore`

1. `rosrun tf static_transform_publisher 0 0 0 0 0 0 darpa map 100`

1. `roslaunch mapping_server mapping_server.launch`

1. `roslaunch mapping_relay mapping_relay.launch`

1. `roslaunch darpa_reporting darpa_reporting.launch`
