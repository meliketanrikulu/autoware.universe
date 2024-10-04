#ifndef PCD_LOADER_HPP_
#define PCD_LOADER_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdLoader : public rclcpp::Node
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  PcdLoader() : Node("pcd_loader")
  {





    get_differential_pcd_maps_service_ = create_service<GetDifferentialPointCloudMap>(
  "pcd_loader_service", std::bind(
                          &PcdLoader::on_service_get_differential_point_cloud_map, this,
                          std::placeholders::_1, std::placeholders::_2));


  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;


  bool on_service_get_differential_point_cloud_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res)
  {
    std::cout<<"xx on_service_get_differential_point_cloud_map"<< req->area.radius<<std::endl;
    // Load the PCD map
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/melike/rosbags/urban_environment/ista_map/ista_dataset_route_3_map/autoware_maps/map_part/pointcloud_map.pcd", *cloud_ptr) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read the PCD file");
        return false;
    }

    // Publish the loaded PCD map
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_ptr, cloud_msg);
    cloud_msg.header.frame_id = "map";
    pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pcd_cgt", 10);
    pcd_publisher_->publish(cloud_msg);
    std::cout<<"xx cloud_msg "<<cloud_msg.width<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud= *cloud_ptr;
    autoware_map_msgs::msg::PointCloudMapCellWithID pcd_map_cell_with_id;
    pcd_map_cell_with_id.cell_id = "0";
    pcd_map_cell_with_id.metadata.min_x = std::numeric_limits<float>::max();
    pcd_map_cell_with_id.metadata.min_y = std::numeric_limits<float>::max();
    pcd_map_cell_with_id.metadata.max_x = std::numeric_limits<float>::lowest();
    pcd_map_cell_with_id.metadata.max_y = std::numeric_limits<float>::lowest();
    for (const auto & point : cloud.points) {
      pcd_map_cell_with_id.metadata.min_x = std::min(pcd_map_cell_with_id.metadata.min_x, point.x);
      pcd_map_cell_with_id.metadata.min_y = std::min(pcd_map_cell_with_id.metadata.min_y, point.y);
      pcd_map_cell_with_id.metadata.max_x = std::max(pcd_map_cell_with_id.metadata.max_x, point.x);
      pcd_map_cell_with_id.metadata.max_y = std::max(pcd_map_cell_with_id.metadata.max_y, point.y);
    }

    RCLCPP_INFO_STREAM(get_logger(), "cloud size: " << cloud.size());
    pcl::toROSMsg(cloud, pcd_map_cell_with_id.pointcloud);

    res->new_pointcloud_with_ids.push_back(pcd_map_cell_with_id);
    res->header.frame_id = "map";
    return true;
  }
};

#endif  // PCD_LOADER_HPP_
