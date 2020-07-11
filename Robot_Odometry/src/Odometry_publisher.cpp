/*odom座標フレームからbase_link座標フレームへの変換とnav_msgs / Odometryメッセージの両方を公開するので、
関連するヘッダーファイルを含める必要があります*/
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char** argv)
{
    // ノードの名前 odometry_publisher
    ros::init(argc, argv, "odometry_publisher");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;
    /*ROSとtfをそれぞれ使用してメッセージを送信できるようにするには、
    ros::Publisherとtf::TransformBroadcasterの両方を作成する必要があります。*/
    // トピックの名前 odom 
    // Queue size 50
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    // 最初は、ロボットがodom座標フレームの原点から始まると仮定
    double x  = 0.0;
    double y  = 0.0;
    double th = 0.0;
    /*ここでは、"base_link "フレームが "oddom "フレーム内を
    x方向に0.1m/s、y方向に-0.1m/s、th方向に0.1rad/sの速度で移動するような速度を設定します。
    これにより、多かれ少なかれ、私たちの偽ロボットは円を描くようになります。*/
    double vx  = 0.1;
    double vy  = -0.1;
    double vth = 0.1;
    // Time
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    // ループ周期の設定 0.01秒間隔で処理を繰り返す
    ros::Rate r(1.0);

    while (ros::ok()) {
        current_time = ros::Time::now();
        // ロボットの速度を与えられた典型的な方法でオドメトリを計算します。
        double dt       = (current_time - last_time).toSec();
        double delta_x  = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y  = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        
        x += delta_x;
        y += delta_y;
        th += delta_th;

        // すべてのオドメトリが6DOFであるため、ヨーから作成されたクォータニオンが必要
        /*オドメトリのヨー値をクォータニオンに変換して、ワイヤを介して送信する必要があります。
        幸い、tfはヨー値からの四元数の簡単な作成と、四元数からのヨー値への簡単なアクセスを可能にする関数を提供します。*/
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        /*tfを介して送信するTransformStampedメッセージを作成します。
        odomフレームからbase_linkフレームへの変換をcurrent_timeに公開したいとします。
        したがって、メッセージのヘッダーとchild_frame_idを適切に設定し、
        必ず親座標フレームとしてodomを、子座標フレームとしてbase_linkを使用します。*/
        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp    = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";

        // オドメトリデータから変換メッセージを入力し、TransformBroadcasterを使用して変換を送信します。
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // 次に、オドメトリーメッセージをROS上で公開します
        nav_msgs::Odometry odom;
        odom.header.stamp    = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        // 位置を設定
        odom.pose.pose.position.x  = x;
        odom.pose.pose.position.y  = y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = odom_quat;
        // 速度を設定
        odom.twist.twist.linear.x  = vx;
        odom.twist.twist.linear.y  = vy;
        odom.twist.twist.angular.z = vth;
        // メッセージを公開
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}



