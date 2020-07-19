#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/transform_datatypes.h"
#include "boost/thread.hpp"

// アクションクライアントの型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "navigation_goal");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;
    // アクションクライアントの宣言
    MoveBaseClient ac("pose_controller", true);
    // アクションサーバが実行されるまで待機
    ac.waitForServer();
    // アクション目標のオブジェクト宣言
    move_base_msgs::MoveBaseGoal goal;
    // ロボットにゴールを送って ２メートル前に移動させる
    //goal.target_pose.header.frame_id    = "base_link";
    goal.target_pose.header.frame_id    = "map";
    goal.target_pose.header.stamp       = ros::Time::now();
    try{
		goal.target_pose.pose.position.x    = atof(argv[1]);
		goal.target_pose.pose.position.y    = atof(argv[2]);
		goal.target_pose.pose.orientation.w = atof(argv[3]);
	   }
	catch(int e){

		goal.target_pose.pose.position.x    = 2.0;
		goal.target_pose.pose.position.y    = 0.2;
		goal.target_pose.pose.orientation.w = 0.1;
	}

    // アクション目標の転送
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Success!");
    } else {
        ROS_INFO("Error!");
    }

    return 0;
}