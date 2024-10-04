#ifndef ROSBAG_PARSER_NODE__ROSBAG_PARSER_NODE_HPP_
#define ROSBAG_PARSER_NODE__ROSBAG_PARSER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <memory>

class RosbagParserNode : public rclcpp::Node {
public:
    RosbagParserNode(const std::string &bag_file_path)
        : Node("rosbag_parser_node") {

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_path;
        storage_options.storage_id = "sqlite3";  // Default is sqlite3 for ROS2

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(storage_options, converter_options);
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud2> get_pc() {
        while (reader_->has_next()) {
            auto bag_message = reader_->read_next();

            if (bag_message->topic_name == "/localization/util/downsample/pointcloud") {
                return deserialize_message<sensor_msgs::msg::PointCloud2>(bag_message);
            }
        }
        return nullptr;
    }

    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> get_pose_with_cov() {
        while (reader_->has_next()) {
            auto bag_message = reader_->read_next();

            if (bag_message->topic_name == "/sensing/gnss/pose_with_covariance") {
                return deserialize_message<geometry_msgs::msg::PoseWithCovarianceStamped>(bag_message);
            }
        }
        return nullptr;
    }

    std::shared_ptr<tf2_msgs::msg::TFMessage> get_tf() {

        while (reader_->has_next()) {
            auto bag_message = reader_->read_next();

            if (bag_message->topic_name == "/tf_static") {
                return deserialize_message<tf2_msgs::msg::TFMessage>(bag_message);
            }
        }
        return nullptr;
    }

private:
    template<typename MessageType>
    std::shared_ptr<MessageType> deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto message = std::make_shared<MessageType>();
        rclcpp::Serialization<MessageType> serializer;
        serializer.deserialize_message(&serialized_msg, message.get());
        return message;
    }

    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

#endif  // ROSBAG_PARSER_NODE__ROSBAG_PARSER_NODE_HPP_
