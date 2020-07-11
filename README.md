# ROS_navigation_wiki
ROS_navigation_wiki
# Robot_setup_tf
単純なフレーム間の座標変換(tf)
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF  
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_setup_tf roscpp tf geometry_msgs
```
ノードのビルド
```bash
cd ~/catkin_ws
catkin_make
```
ROSの起動
```bash
roscore
```
tfのbroadcasterの起動
```bash
rosrun robot_setup_tf tf_broadcaster
```
tfのlistenerの起動  
```bash
rosrun robot_setup_tf tf_listener
```
