Build this package with catkin. Make sure to source your workspace at every command execution below.

Start the Thorvald/Husky robots and `/navigate_to_pose` action server.

> roslaunch imrt_navigate_to_pose thorvald_nav.launch

You can send action goals to `/navigate_to_pose` action server with `axclient.py` GUI.

> rosrun actionlib axclient.py /navigate_to_pose imrt_navigate_to_pose/NavigateToPoseAction

Or you can send action goals to `/navigate_to_pose` action server with client node;

> rosrun imrt_navigate_to_pose action_client