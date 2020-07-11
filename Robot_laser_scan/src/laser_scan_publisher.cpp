#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char** argv)
{
    // ノードの名前 laser_scan_publisher
    ros::init(argc, argv, "laser_scan_publisher");
    // ROSシステムと通信を行うためのノードハンドルの宣言
    ros::NodeHandle nh;
    // トピックの名前 scan
    // Queue size 50
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    /*ここでは、スキャンに使用するダミーデータのためのストレージを設定しています。
    実際のアプリケーションでは、レーザードライバからこのデータを取得します。*/
    unsigned int num_readings = 100;
    double laser_frequency    = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    int count = 0;
    ros::Rate r(1.0);

    while (ros::ok()) {
        // ダミーデータ作成
        for (unsigned int i = 0; i < num_readings; i++) {
            ranges[i]      = count;
            intensities[i] = 100 + count;
        }

        ros::Time scan_time = ros::Time::now();

        // ダミーのレーザーデータには、1秒ごとに1つずつ増加する値を入力します。
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "laser_frame";
        // スキャンの開始角度[rad]
        scan.angle_min = -1.57;
        // スキャンの終了角度[rad]
        scan.angle_max = 1.57;
        // 測定間の角度距離[rad]
        scan.angle_increment = 3.14 / num_readings;
        // 測定間の時間[秒]
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        // 最小範囲値[m]
        scan.range_min = 0.0;
        // 最大範囲値[m]
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        // scan_msgs::LaserScanメッセージを作成し、有線で送信するための準備として生成したデータを埋めます。
        for (unsigned int i = 0; i < num_readings; i++) {
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }
        
        // メッセージをROS経由で公開します。
        scan_pub.publish(scan);
        ++count;
        r.sleep();
    }
}