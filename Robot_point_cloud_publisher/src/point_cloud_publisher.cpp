#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

int main(int argc, char** argv)
{
    // ノードの名前 point_cloud_publisher
    ros::init(argc, argv, "point_cloud_publisher");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;
    // トピックの名前 cloud
    // Queue size 50
    // PointCloudメッセージをネットワーク経由で送信するために使用するros::Publisherを作成
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 50);

    unsigned int num_points = 100;

    int count = 0;
    ros::Rate r(1.0);

    while (ros::ok()) {
        // 送信するPointCloudメッセージのヘッダーに、関連するフレームとタイムスタンプ情報を入力
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp    = ros::Time::now();
        cloud.header.frame_id = "sensor_frame";
        cloud.points.resize(num_points);

        // 点群の点の数を設定して、ダミーデータを入力できるようにします。
        // intensityと呼ばれるチャネルをPointCloudに追加し、クラウドに配置するポイントの数に合わせてサイズを変更します。
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[1].values.resize(num_points);

        // 点群の偽のデータを生成します 
        for (unsigned int i = 0; i < num_points; i++) {
            cloud.points[i].x = 1 + count;
            cloud.points[i].y = 2 + count;
            cloud.points[i].z = 3 + count;
            cloud.channels[0].values[i] = 100 + count;
        }

        // ROSを使用してポイントクラウドを公開
        cloud_pub.publish(cloud);
        ++count;
        r.sleep();
    }
}