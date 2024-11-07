#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("judge_rtk_status") {
        // サブスクライバを作成して、コールバック関数を設定
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_gps_node/fix", 10,
            std::bind(&GPSSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // status フィールドの status が 2 かを確認
        if (msg->status.status == 2) {
            RCLCPP_INFO(this->get_logger(), "Status is 2");
            RCLCPP_INFO(this->get_logger(), "Latitude: %f, Longitude: %f, Altitude: %f",
                        msg->latitude, msg->longitude, msg->altitude);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
