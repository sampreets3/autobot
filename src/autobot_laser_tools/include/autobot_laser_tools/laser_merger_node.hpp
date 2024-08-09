#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class LaserScanMerger: public rclcpp::Node
{
public:
    LaserScanMerger();

private:
    /**
     * @brief   Subscriber callback function for the front scanner.
     * 
     * @param   scan    The scan message published by the laser scanner.
     * 
     * @return  None.
     */
    void FrontScanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan);


    /**
     * @brief   Subscriber callback function for the rear scanner.
     * 
     * @param   scan    The scan message published by the laser scanner.
     * 
     * @return  None.
     */
    void RearScanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan);


    /**
     * @brief   Timer callback to publish the merged scan.
     */
    void TimerCallback();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_1;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_2;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_merged_pub;

    sensor_msgs::msg::LaserScan front_scan;
    sensor_msgs::msg::LaserScan rear_scan;
    
    std::string front_scan_topic_name;
    std::string rear_scan_topic_name;
    std::string merged_scan_topic_name;
    std::string merged_scan_frame;

};