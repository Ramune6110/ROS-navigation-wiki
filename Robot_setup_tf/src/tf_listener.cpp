/*ROSを介してbase_laser → base_link変換を公​​開するノードを作成しました。
次に、その変換を使用して「base_laser」フレーム内のポイントを取得し、
それを「base_link」フレーム内のポイントに変換するノードを作成*/
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

// base_laserフレーム内のポイントを取得し、それをbase_linkフレームに変換する関数を作成
// この関数は、プログラムのmainで作成されたros::Timerのコールバックとして機能し、毎秒起動します。
void transformPoint(const tf::TransformListener& listener)
{
    // base_laserフレームの中に、base_linkフレームに変換したいポイントを作成
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_laser";
    // ここでは、単純な例で利用可能な最新の変換を使用します。
    laser_point.header.stamp = ros::Time();
    // 空間の任意の点
    laser_point.point.x = 1.0;
    laser_point.point.y = 0.2;
    laser_point.point.z = 0.0;

    /*"base_laser "フレームに点があるので、それを "base_link "フレームに変換したいと思います。
    そのためには、TransformListenerオブジェクトを使用し、3つの引数を指定してtransformPoint()を呼び出します：
    ポイントを変換したいフレームの名前（この例では "base_link"）、変換するポイント、変換したポイントの格納場所です。
    つまり、transformPoint()を呼び出した後、base_pointは、以前のlaser_pointと同じ情報を保持していますが、
    現在は "base_link "フレームにのみ保持されています。*/
    try {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", laser_point, base_point);
        ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 laser_point.point.x, laser_point.point.y, laser_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

    } 
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    // ノードの名前 robot_tf_listener
    ros::init(argc, argv, "robot_tf_listener");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;

    tf::TransformListener listener(ros::Duration(10));

    // 毎秒1回ポイントを変換します
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    ros::spin();
}