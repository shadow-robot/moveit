# MoveIt! ROS Warehouse

This package provides methods to access the MoveIt! warehouse. 


## Export to text

An executable is available to allow to export the scene and queries that are available in the Moveit! warehouse to text files:

```
rosrun moveit_ros_warehouse moveit_warehouse_save_as_text
```

It has different parameters that can be specified:

* `help`: Show help message
* `host`: Host for the DB
* `port`: Port for the DB
* `output_directory`, Directory to save the generated files
* `scene`: Saves the scenes available in the warehouse. It generates .scene files. If it is not specified the queries are saved.
* `cartesian`: Save queries in cartesian space (start and end pose of eef). If not specified they will be saved in joint space by default.
* `eef`, Specify the end effector (Only needed when the cartesian option is set). Default: last link. 
* `group_prefix`, Specify the group prefix you'd like to plan with. This is useful if you want to save the queries only for the arm and it has a prefix, so only those joints will be considered (e.g. "ra")

To be able to use this command, the robot should be launched in a terminal.

Here are a few examples of how to use them:
* Example to export the scenes:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --scene --output_directory /tmp/scene
  ```
* Example to export the queries in joint space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra
  ```
* Example to export the queries in cartesian space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra --cartesian --eef ra_wrist_3_link
  ```

## Import from text

