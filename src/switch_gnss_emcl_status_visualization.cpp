#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" //gnss_pose
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"          // クオータニオン操作
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // tf2::toMsg などの変換
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("switch_gnss_emcl_status_visualization") {
        // サブスクライバを作成して、コールバック関数を設定
        /*subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_gps_node/fix", 10,
            std::bind(&GPSSubscriber::topic_callback, this, std::placeholders::_1));*/
        gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&GPSSubscriber::pose_callback, this, _1));
        odrive_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odrive_odom", 10, std::bind(&GPSSubscriber::odometry_callback, this, _1));
        emcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mcl_pose", 10, std::bind(&GPSSubscriber::emcl_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("status_flug_marker", 10);
        switch_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("switch_odom", 10);
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 2);

        odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        current_time = this->get_clock()->now();

        timer_ = this->create_wall_timer(50ms, std::bind(&GPSSubscriber::timer_callback, this));
        start_roof_x = 0.49468;
        start_roof_y = 0.0809078;

        goal_roof_x = 110.068;
        goal_roof_y = -67.6848;


        // 静的な変換を送信するタイマー
        //static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        //send_static_transform();
    }

private:
    void timer_callback()
    {
        publish_marker();
        if(emcl_flag && gnss_flag && odrive_flag){
            RCLCPP_INFO(this->get_logger(), "run node");
            current_time = this->get_clock()->now();

            if(status_flag){
                x = gnss_x;
                y = gnss_y;
                double dgx = x - pre_gnss_x;
                double dgy = y - pre_gnss_y;
                double dist_gnss = std::hypot(dgx, dgy);
                RCLCPP_INFO(this->get_logger(), "vx, dist_gnss:%lf, %lf",vx, dist_gnss);
                if(vx > 0.2 && dist_gnss > 0.001 && dist_gnss < 3.0){
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

                    RCLCPP_INFO(this->get_logger(), "temp_gnss_yaw: %lf, %lf", pre_yaw, temp_gnss_yaw);
                    if(std::abs(pre_yaw - temp_gnss_yaw) > M_PI/5){
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
                
                double dgpx = gnss_x - start_roof_x;
                double dgpy = gnss_y - start_roof_y;
                double dist_roof = std::hypot(dgpx, dgpy);
                //屋根下地点に到着した場合
                if(dist_roof < 0.3){
                    status_flag = false;
                    // initialpose をパブリッシュ
                    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
                    initial_pose_msg.header.stamp = current_time;
                    initial_pose_msg.header.frame_id = "map";

                    initial_pose_msg.pose.pose.position.x = x;
                    initial_pose_msg.pose.pose.position.y = y;
                    initial_pose_msg.pose.pose.position.z = 0.0;

                    // yaw をクオータニオンに変換
                    tf2::Quaternion quat;
                    quat.setRPY(0, 0, yaw);
                    initial_pose_msg.pose.pose.orientation = tf2::toMsg(quat);

                    // 共分散行列（例: 位置と姿勢に適当な確信度を設定）
                    for (int i = 0; i < 36; ++i) {
                        initial_pose_msg.pose.covariance[i] = 0.0;
                    }

                    initial_pose_pub_->publish(initial_pose_msg);
                }
                odom_x = x;
                odom_y = y;
                odom_yaw = yaw;
                
                gnss_count ++;
                switch_flag = true;
            }else{
                if(switch_flag){
                    odom_x = x;
                    odom_y = y;
                    odom_yaw = yaw;
                    switch_yaw = yaw - odrive_yaw;
                }

                x = emcl_x;
                y = emcl_y;
                yaw = emcl_yaw;
                RCLCPP_INFO(this->get_logger(), "emcl_yaw: %lf", yaw);

                auto [tx, ty] = transform_to_robot_frame(diff_odrive_x, diff_odrive_y, switch_yaw);
                odom_x += tx;
                odom_y += ty;
                odom_yaw += diff_odrive_yaw;
                switch_flag = false;
            }

            tf2::Quaternion odom_quat;
            odom_quat.setRPY(0, 0, yaw);  // ロール、ピッチ、ヨーをセット
            geometry_msgs::msg::Quaternion odom_quat_msg =
                tf2::toMsg(odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換

            
            
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

            tf2::Quaternion tf_quat;
            tf_quat.setRPY(0, 0, odom_yaw);  // ロール、ピッチ、ヨーをセット
            geometry_msgs::msg::Quaternion tf_quat_msg =
                tf2::toMsg(tf_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換

            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = odom_x;
            odom_trans.transform.translation.y = odom_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf_quat_msg;
            odom_broadcaster->sendTransform(odom_trans);

            RCLCPP_INFO(this->get_logger(), "odom_position: %lf, %lf, %lf", odom_x, odom_y, odom_yaw);
            RCLCPP_INFO(this->get_logger(), "odrive_position: %lf, %lf, %lf", odrive_x, odrive_y, odrive_yaw);

            pre_yaw = yaw;
        }

    }

    std::pair<double, double> transform_to_robot_frame(double trans_x, double trans_y, double trans_yaw){
        double x_in_robot = trans_x * cos(trans_yaw) - trans_y * sin(trans_yaw);
        double y_in_robot = trans_y * cos(trans_yaw) + trans_x * sin(trans_yaw);
        return {x_in_robot, y_in_robot};
    }

    void publish_marker() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "base_link"; // 適切な固定フレームを設定
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "status_flag_ns";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // テキストタイプ
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 表示するテキスト
        marker.text = status_flag ? "GPS mode" : "SLAM mode";

        // 位置とスケールを設定
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 2.0; // カメラから見える位置に調整
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5; // テキストサイズ
        marker.color.a = 1.0; // アルファ値（透明度）
        marker.color.r = 1.0; // 赤
        marker.color.g = 1.0; // 緑
        marker.color.b = 1.0; // 青

        marker_pub_->publish(marker);
    }

    void emcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        const auto &emcl_pose = msg->pose.pose;
        emcl_x = emcl_pose.position.x;
        emcl_y = emcl_pose.position.y;
        tf2::Quaternion quat;
        tf2::fromMsg(emcl_pose.orientation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll_tmp, pitch_tmp, yaw_tmp;
        mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

        emcl_yaw = yaw_tmp;

        if(!emcl_flag){
            pre_emcl_x = emcl_x;
            pre_emcl_y = emcl_y;
            pre_emcl_yaw = emcl_yaw;
        }

        //移動量を計算
        diff_emcl_x = emcl_x - pre_emcl_x;
        diff_emcl_y = emcl_y - pre_emcl_y;
        dist_emcl = std::hypot(diff_emcl_x, diff_emcl_y);

        diff_emcl_yaw = emcl_yaw - pre_emcl_yaw;

        double dgpx = emcl_x - goal_roof_x;
        double dgpy = emcl_x - goal_roof_y;
        double dist_roof = std::hypot(dgpx, dgpy);
        if(dist_roof < 0.5){
            status_flag = false;
        }

        pre_emcl_x = emcl_x;
        pre_emcl_y = emcl_y;
        pre_emcl_yaw = emcl_yaw;
        emcl_flag = true;

    }
    /*
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
    }*/

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        gnss_x = msg->pose.position.x;
        gnss_y = msg->pose.position.y;
        double init_yaw = M_PI;
        if (!gnss_flag){
            tf2::Quaternion odom_quat;
            odom_quat.setRPY(0, 0, init_yaw);  // ロール、ピッチ、ヨーをセット
            geometry_msgs::msg::Quaternion odom_quat_msg =
                tf2::toMsg(odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換
            
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = gnss_x;
            odom_trans.transform.translation.y = gnss_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat_msg;
            odom_broadcaster->sendTransform(odom_trans);
        }
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
        
        if (!odrive_flag){
            tf2::Quaternion odrive_odom_quat;
            odrive_odom_quat.setRPY(0, 0, odrive_yaw);  // ロール、ピッチ、ヨーをセット
            geometry_msgs::msg::Quaternion odrive_odom_quat_msg =
                tf2::toMsg(odrive_odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換
            
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = odrive_x;
            odom_trans.transform.translation.y = odrive_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odrive_odom_quat_msg;
            odom_broadcaster->sendTransform(odom_trans);
        }

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
        odrive_flag = true;
    }
    /*
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
    */

    //rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odrive_odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr emcl_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr switch_odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    //std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time current_time;

    double x = 0.0, y = 0.0, gnss_x, gnss_y, odrive_x, odrive_y, odrive_yaw, yaw = 0.0, vx, vth, temp_gnss_yaw, switch_yaw;

    double emcl_x, emcl_y, emcl_yaw, pre_emcl_x = 0.0, pre_emcl_y = 0.0, pre_emcl_yaw = 0.0, 
        diff_emcl_x = 0.0, diff_emcl_y = 0.0, diff_emcl_yaw = 0.0, dist_emcl;

    double pre_odrive_x = 0.0, pre_odrive_y = 0.0, pre_odrive_yaw = 0.0, diff_odrive_x = 0.0, diff_odrive_y = 0.0, diff_odrive_yaw = 0.0, pre_gnss_x = 0.0, pre_gnss_y = 0.0, pre_yaw = 0.0;

    double start_roof_x, start_roof_y, goal_roof_x, goal_roof_y;

    bool status_flag = false, ublox_flag = false, gnss_flag = false, odrive_flag = false, emcl_flag = false, switch_flag = true;
    int gnss_count = 0;
    double odom_x, odom_y, odom_yaw;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
