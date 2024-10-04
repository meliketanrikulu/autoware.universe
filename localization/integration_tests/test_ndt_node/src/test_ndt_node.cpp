#include "test_ndt_node/test_ndt_node.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

TestNDTNode::TestNDTNode() : Node("rosbag_handler_node")
{
  std::string input_bag_path = "/home/melike/rosbags/urban_shared_data/rosbag2_2024_09_11-17_53_54/rosbag2_2024_09_11-17_53_54_0.db3";

  // Generate a unique output path using timestamp
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "/home/melike/rosbags/urban_shared_data/write_new_bag/test_"
     << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");

  std::string output_bag_path = ss.str();

  // Setup storage options for reader
  rosbag2_storage::StorageOptions reader_storage_options;
  reader_storage_options.uri = input_bag_path;
  reader_storage_options.storage_id = "sqlite3";  // Default is sqlite3 for ROS2

  // Initialize the reader
  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader_->open(reader_storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open input bag file: %s", e.what());
    throw;
  }

  // Initialize the writer
  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer_->open(output_bag_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open output bag file: %s", e.what());
    throw;
  }

  // Get topic information from the input bag and create them in the output bag
  auto topics_and_types = reader_->get_all_topics_and_types();

  RCLCPP_INFO(this->get_logger(), "Topics in the input bag:");
  for (const auto &topic_info : topics_and_types) {
    RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic_info.name.c_str(), topic_info.type.c_str());

    // Only create topics we are interested in
    if (topic_info.name == "/pandar_points" ||
        topic_info.name == "/applanix/lvx_client/gnss/fix" ||
        topic_info.name == "/tf_static" ||
        topic_info.name == "/localization/twist_estimator/twist_with_covariance" ||
        topic_info.name == "/clock" ||
        topic_info.name == "/applanix/lvx_client/odom" ||
        topic_info.name == "/applanix/lvx_client/imu_raw" ||
        topic_info.name == "/applanix/lvx_client/autoware_orientation") {
      writer_->create_topic(topic_info);
    }
  }

  while (reader_->has_next()) {
    auto bag_message = reader_->read_next();
    process_message(bag_message);
  }

  RCLCPP_INFO(this->get_logger(), "Finished processing bag files.");
}

void TestNDTNode::process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  std::string topic_name = bag_message->topic_name;
  rclcpp::Time timestamp(bag_message->time_stamp);

  if (topic_name == "/pandar_points") {
    auto pointcloud_msg = deserialize_message<sensor_msgs::msg::PointCloud2>(bag_message);
    handle_pointcloud(pointcloud_msg, timestamp);
  } else if (topic_name == "/sensin/gnss/pose_with_covariance") {
    auto nav_sat_fix_msg = deserialize_message<geometry_msgs::msg::PoseWithCovarianceStamped>(bag_message);
    handle_pose_with_cov(nav_sat_fix_msg, timestamp);
  } else if (topic_name == "/tf_static") {
    auto tf_static_msg = deserialize_message<tf2_msgs::msg::TFMessage>(bag_message);
    handle_tf_static(tf_static_msg, timestamp);
  } else if (topic_name == "/localization/twist_estimator/twist_with_covariance") {
    auto twist_msg = deserialize_message<geometry_msgs::msg::TwistWithCovarianceStamped>(bag_message);
    handle_twist_with_covariance(twist_msg, timestamp);
  } else if (topic_name == "/clock") {
    auto clock_msg = deserialize_message<rosgraph_msgs::msg::Clock>(bag_message);
    handle_clock(clock_msg, timestamp);
  } else {
    // Optionally, log unhandled topics
    RCLCPP_DEBUG(this->get_logger(), "Ignoring topic: %s", topic_name.c_str());
  }


}

template<typename MessageType>
std::shared_ptr<MessageType> TestNDTNode::deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
  auto message = std::make_shared<MessageType>();
  rclcpp::Serialization<MessageType> serializer;
  serializer.deserialize_message(&serialized_msg, message.get());
  return message;
}


void TestNDTNode::handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud2 message received with width: %d, height: %d", msg->width, msg->height);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  // serialize_and_write_message("/pandar_points", msg, timestamp);
}

void TestNDTNode::handle_pose_with_cov(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg, const rclcpp::Time &timestamp)
{
  // RCLCPP_INFO(this->get_logger(), "NavSatFix received: [%.2f, %.2f, %.2f]",
  //             msg->latitude, msg->longitude, msg->altitude);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  // serialize_and_write_message("/applanix/lvx_client/gnss/fix", msg, timestamp);
}

void TestNDTNode::handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg, const rclcpp::Time &timestamp)
{
  for (const auto &transform : msg->transforms) {
    RCLCPP_INFO(this->get_logger(), "Static transform received from %s to %s",
                transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
  }

  // Optionally process the message here

  // Serialize and write the message to the output bag
  // serialize_and_write_message("/tf_static", msg, timestamp);
}

void TestNDTNode::handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "TwistWithCovarianceStamped received: [linear.x: %.2f, angular.z: %.2f]",
              msg->twist.twist.linear.x, msg->twist.twist.angular.z);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  // serialize_and_write_message("/localization/twist_estimator/twist_with_covariance", msg, timestamp);
}

void TestNDTNode::handle_clock(const std::shared_ptr<rosgraph_msgs::msg::Clock> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "Clock message received: %d", msg->clock.sec);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  // serialize_and_write_message("/clock", msg, timestamp);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNDTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
