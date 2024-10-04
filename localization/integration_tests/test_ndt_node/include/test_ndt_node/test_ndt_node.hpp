#ifndef TEST_NDT_NODE__TEST_NDT_NODE_HPP_
#define TEST_NDT_NODE__TEST_NDT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "geometry_msgs//msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>

class TestNDTNode : public rclcpp::Node
{
public:
    TestNDTNode();

private:
    void process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    template<typename MessageType>
    std::shared_ptr<MessageType> deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    void handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const rclcpp::Time &timestamp);
    void handle_pose_with_cov(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg, const rclcpp::Time &timestamp);
    void handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg, const rclcpp::Time &timestamp);
    void handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, const rclcpp::Time &timestamp);
    void handle_clock(const std::shared_ptr<rosgraph_msgs::msg::Clock> &msg, const rclcpp::Time &timestamp);
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

#endif // TEST_NDT_NODE__TEST_NDT_NODE_HPP_
