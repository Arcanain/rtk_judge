#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" //gnss_pose
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"          // クオータニオン操作
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // tf2::toMsg などの変換

using std::placeholders::_1;
using namespace std::chrono_literals;

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("judge_rtk_status_origin") {
        // サブスクライバを作成して、コールバック関数を設定
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_gps_node/fix", 10,
            std::bind(&GPSSubscriber::topic_callback, this, std::placeholders::_1));
        gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&GPSSubscriber::pose_callback, this, _1));
        odrive_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odrive_odom", 10, std::bind(&GPSSubscriber::odometry_callback, this, _1));

        switch_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("switch_odom", 10);

        odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        current_time = this->get_clock()->now();

        timer_ = this->create_wall_timer(50ms, std::bind(&GPSSubscriber::timer_callback, this));
        // 静的な変換を送信するタイマー
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        send_static_transform();
    }

private:
    void timer_callback()
    {
        if(ublox_flag && gnss_flag && odrive_flag){

            current_time = this->get_clock()->now();

            if(status_flag){
                x = gnss_x;
                y = gnss_y;
                double dgx = x - pre_gnss_x;
                double dgy = y - pre_gnss_y;
                double dist_gnss = std::hypot(dgx, dgy);
                RCLCPP_INFO(this->get_logger(), "vx, dist_gnss:%lf, %lf",vx, dist_gnss);
                if(vx > 0.2 && dist_gnss > 0.03 && dist_gnss < 3.0){
                    temp_gnss_yaw = std::atan2(dgy, dgx);
                    
                    while (temp_gnss_yaw > 2*M_PI){
                        temp_gnss_yaw = temp_gnss_yaw - 2 * M_PI;
                    }
                    while (temp_gnss_yaw < - 2*M_PI){
                        temp_gnss_yaw = temp_gnss_yaw + 2 * M_PI;
                    }
                    while (pre_yaw > 2*M_PI){
                        pre_yaw = pre_yaw - 2 * M_PI;
                    }
                    while (pre_yaw < - 2*M_PI){
                        pre_yaw = pre_yaw + 2 * M_PI;
                    }

                    RCLCPP_INFO(this->get_logger(), "temp_gnss_yaw: %lf, %lf, %lf", pre_yaw, temp_gnss_yaw, std::abs(std::abs(pre_yaw) - std::abs(temp_gnss_yaw)));
                    RCLCPP_INFO(this->get_logger(), "M_PI/3: %lf", M_PI/3);
                    if(std::abs(std::abs(pre_yaw) - std::abs(temp_gnss_yaw)) > M_PI/10){
                        yaw += diff_odrive_yaw;
                        RCLCPP_INFO(this->get_logger(), "odrive_yaw: %lf", yaw);
                    }else{
                        yaw = temp_gnss_yaw;
                        RCLCPP_INFO(this->get_logger(), "fix_yaw:%lf",yaw);
                    }
                }else{
                    yaw += diff_odrive_yaw;
                    RCLCPP_INFO(this->get_logger(), "odrive_yaw: %lf", yaw);
                }
                if(gnss_count % 20 == 0){
                    pre_gnss_x = x;
                    pre_gnss_y = y;
                }
                pre_yaw = temp_gnss_yaw;
                gnss_count ++;
                switch_flag = true;
            }else{
                if(switch_flag){
                    switch_yaw = yaw - odrive_yaw;
                }
                auto [tx, ty] = transform_to_robot_frame(diff_odrive_x, diff_odrive_y, switch_yaw);
                x += tx;
                y += ty;
                yaw += diff_odrive_yaw;
                RCLCPP_INFO(this->get_logger(), "odrive_yaw: %lf", yaw);
                switch_flag = false;
            }

            tf2::Quaternion odom_quat;
            odom_quat.setRPY(0, 0, yaw);  // ロール、ピッチ、ヨーをセット
            geometry_msgs::msg::Quaternion odom_quat_msg =
                tf2::toMsg(odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat_msg;

            odom_broadcaster->sendTransform(odom_trans);

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat_msg;

            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = vth;

            switch_odom_pub->publish(odom);
            //pre_yaw = yaw;
        }

    }

    std::pair<double, double> transform_to_robot_frame(double trans_x, double trans_y, double trans_yaw){
        double x_in_robot = trans_x * cos(trans_yaw) - trans_y * sin(trans_yaw);
        double y_in_robot = trans_y * cos(trans_yaw) + trans_x * sin(trans_yaw);
        return {x_in_robot, y_in_robot};
    }

    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // status フィールドの status が 2 かを確認
        if (msg->status.status == 2) {
            RCLCPP_INFO(this->get_logger(), "Status is 2");
            //RCLCPP_INFO(this->get_logger(), "Latitude: %f, Longitude: %f, Altitude: %f", msg->latitude, msg->longitude, msg->altitude);
            status_flag = true;
        }else{
            status_flag = false;
        }
        ublox_flag = true;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        gnss_x = msg->pose.position.x;
        gnss_y = msg->pose.position.y;
        gnss_flag = true;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odrive_x = msg->pose.pose.position.x;
        odrive_y = msg->pose.pose.position.y;
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll_tmp, pitch_tmp, yaw_tmp;
        mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

        odrive_yaw = yaw_tmp;
        //RCLCPP_INFO(this->get_logger(), "odrive_yaw: %lf", odrive_yaw);
        odrive_flag = true;

        // 並進速度(vx)と角速度(vth)の取得
        vx = msg->twist.twist.linear.x;
        vth = msg->twist.twist.angular.z;

        //移動量を計算
        diff_odrive_x = odrive_x - pre_odrive_x;
        diff_odrive_y = odrive_y - pre_odrive_y;
        diff_odrive_yaw = odrive_yaw - pre_odrive_yaw;

        pre_odrive_x = odrive_x;
        pre_odrive_y = odrive_y;
        pre_odrive_yaw = odrive_yaw;
    }

    void send_static_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform_stamped;
        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "map";
        static_transform_stamped.child_frame_id = "odom";
        static_transform_stamped.transform.translation.x = 0.0;
        static_transform_stamped.transform.translation.y = 0.0;
        static_transform_stamped.transform.translation.z = 0.0;
        static_transform_stamped.transform.rotation.x = 0.0;
        static_transform_stamped.transform.rotation.y = 0.0;
        static_transform_stamped.transform.rotation.z = 0.0;
        static_transform_stamped.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_transform_stamped);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odrive_odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr switch_odom_pub;


    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time current_time;

    double x, y, gnss_x, gnss_y, odrive_x, odrive_y,odrive_yaw, yaw, vx, vth, temp_gnss_yaw, switch_yaw;
    double pre_odrive_x = 0.0, pre_odrive_y = 0.0, pre_odrive_yaw = 0.0, diff_odrive_x = 0.0, diff_odrive_y = 0.0, diff_odrive_yaw = 0.0, pre_gnss_x = 0.0, pre_gnss_y = 0.0, pre_yaw = 0.0;
    bool status_flag = true, ublox_flag = false, gnss_flag = false, odrive_flag = false, switch_flag = true;
    int gnss_count = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}