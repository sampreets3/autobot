#include "rclcpp/rclcpp.hpp"
#include "autobot_laser_tools/laser_merger_node.hpp"

LaserScanMerger::LaserScanMerger(): Node("laserscan_merger")
{
    this->declare_parameter("scan_1", "front_scan");
    this->declare_parameter("scan_2", "rear_scan");
    this->declare_parameter("merged_scan_frame", "laser_merged");
    this->declare_parameter("merged_scan_topic", "scan");

    this->front_scan_topic_name = this->get_parameter("scan_1").as_string();
    this->rear_scan_topic_name = this->get_parameter("scan_2").as_string();
    this->merged_scan_topic_name = this->get_parameter("merged_scan_topic").as_string();
    this->merged_scan_frame = this->get_parameter("merged_scan_frame").as_string();

    this->scan_subscriber_1 = this->create_subscription<sensor_msgs::msg::LaserScan>(front_scan_topic_name, 10, std::bind(&LaserScanMerger::FrontScanSubscriber, this, std::placeholders::_1));
    this->scan_subscriber_2 = this->create_subscription<sensor_msgs::msg::LaserScan>(rear_scan_topic_name, 10, std::bind(&LaserScanMerger::RearScanSubscriber, this, std::placeholders::_1));

    this->scan_merged_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(merged_scan_topic_name, 10);
}

void LaserScanMerger::FrontScanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    this->front_scan = *scan;
}

void LaserScanMerger::RearScanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    this->rear_scan = *scan;
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}