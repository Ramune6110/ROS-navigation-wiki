# ROS_navigation_wiki
ROS navigationに関する基本事項についてのまとめ
# Robot_tf
単純なフレーム間の座標変換(tf)
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF  
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_tf roscpp tf geometry_msgs
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
rosrun Robot_tf tf_broadcaster
```
tfのlistenerの起動  
```bash
rosrun Robot_tf tf_listener
```
# Robot_Odometry
オドメトリの情報を取得
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_Odometry roscpp tf nav_msgs
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
publisherの起動
```bash
rosrun Robot_Odometry Odometry_publisher
```
トピックリストの確認 
```bash
rostopic list
```
オドメトリ情報の確認
```bash
rostopic echo /odom
```
# Robot_laser_scan
センサーデータの取得
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_laser_scan roscpp sensor_msgs
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
publisherの起動
```bash
rosrun Robot_laser_scan laser_scan_publisher
```
トピックリストの確認 
```bash
rostopic list
```
センサーデータ情報の確認
```bash
rostopic echo /scan
```
# Robot_point_cloud_publisher
クラウドポイントの取得
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_point_cloud_publisher roscpp sensor_msgs
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
publisherの起動
```bash
rosrun Robot_point_cloud_publisher point_cloud_publisher
```
トピックリストの確認 
```bash
rostopic list
```
センサーデータ情報の確認
```bash
rostopic echo /cloud
```
# Robot_navigation_goal
goal(位置と姿勢)を与える
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_navigation_goal actionlib move_base_msgs roscpp tf
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
publisherの起動
```bash
rosrun Robot_navigation_goal navigation_goal
```
トピックリストの確認 
```bash
rostopic list
```
# Robot_2D_navigation
navigationの基礎
## wiki
http://wiki.ros.org/navigation/Tutorials/RobotSetup#Robot_Setup
## Environment
Ubuntu18.04 ROS melodic
## Procedure
パッケージの作成
```bash
cd ~/catkin_ws/src
catkin_create_pkg Robot_2D_navigation move_base
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
roslaunchの起動
```bash
roslaunch Robot_2D_navigation robot_navigation.launch
```
トピックリストの確認 
```bash
rostopic list
```
rqt graphの確認
```bash
rqt_graph
```
[!rosgraph](https://github.com/Ramune6110/ROS_navigation_wiki/blob/master/Robot_2D_navigation/figure/rosgraph.png)
