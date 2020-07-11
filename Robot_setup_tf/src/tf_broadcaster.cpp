// ROS 経由でbase_laser → base_link変換をブロードキャストする作業を行うノードを作成
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv)
{
    // ノードの名前 robot_tf_broadcaster
    ros::init(argc, argv, "robot_tf_broadcaster");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;
    // rate
    ros::Rate r(100);

    /*ここでは、後でbase_link → base_laser変換を
    ワイヤ経由で送信するために使用する
    TransformBroadcasterオブジェクトを作成*/
    tf::TransformBroadcaster broadcaster;

    /*ここで本当の作業が行われます。
    TransformBroadcasterでトランスフォームを送信するには、5つの引数が必要です。
    これは、2つの座標フレームの間で発生する必要がある任意の回転に対して、btQuaternionで指定されます。
    この場合、回転を適用したくないので、ピッチ、ロール、ヨーの値がゼロに等しいものから
    構築されたbtQuaternionを送ります。次に、適用したい任意の並進を表す btVector3 を送ります。
    しかし、平行移動を適用したいので、ロボットベースからのレーザーのxオフセット10cmとzオフセット20cmに対応するbtVector3を作成します。
    三つ目は、公開される変換にタイムスタンプを与える必要があるので、ros::Time::now()でスタンプを押します。
    第四に、作成するリンクの親ノードの名前を渡す必要があります。
    第五に、作成しているリンクの子ノードの名前を渡す必要があります。*/
    while (ros::ok()) {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(), "base_link", "base_laser"
            )
        );
        r.sleep();
    }
}