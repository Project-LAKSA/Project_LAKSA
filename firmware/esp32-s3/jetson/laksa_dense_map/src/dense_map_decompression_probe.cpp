#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/srv/get_map.hpp>

using namespace std::chrono_literals;

namespace
{
std::size_t matrixBytes(const cv::Mat & matrix)
{
  return matrix.total() * matrix.elemSize();
}

bool finitePoint(const pcl::PointXYZRGB & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dense_map_decompression_probe");
  const auto service_name = node->declare_parameter<std::string>(
    "map_data_service", "/probe/rtabmap/get_map_data");
  const int required_nodes = node->declare_parameter<int>("required_nodes", 10);
  const int depth_decimation = node->declare_parameter<int>("depth_decimation", 4);
  const double depth_min = node->declare_parameter<double>("depth_min", 0.3);
  const double depth_max = node->declare_parameter<double>("depth_max", 8.0);

  if (required_nodes < 10 || depth_decimation < 1 || depth_min < 0.0 || depth_max <= depth_min) {
    RCLCPP_ERROR(node->get_logger(), "Invalid probe parameters");
    rclcpp::shutdown();
    return 2;
  }

  auto client = node->create_client<rtabmap_msgs::srv::GetMap>(service_name);
  if (!client->wait_for_service(15s)) {
    RCLCPP_ERROR(node->get_logger(), "PROBE_FAIL service_unavailable=%s", service_name.c_str());
    rclcpp::shutdown();
    return 3;
  }

  auto request = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
  request->global_map = false;
  request->optimized = true;
  request->graph_only = false;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 60s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "PROBE_FAIL service_timeout=%s", service_name.c_str());
    rclcpp::shutdown();
    return 4;
  }

  int tested = 0;
  int passed = 0;
  const auto response = future.get();
  for (const auto & node_message : response->data.nodes) {
    if (tested >= required_nodes) {
      break;
    }
    rtabmap::Signature signature = rtabmap_conversions::nodeFromROS(node_message);
    auto & data = signature.sensorData();
    const std::size_t rgb_compressed = matrixBytes(data.imageCompressed());
    const std::size_t depth_compressed = matrixBytes(data.depthOrRightCompressed());
    if (rgb_compressed == 0 || depth_compressed == 0) {
      continue;
    }

    ++tested;
    cv::Mat rgb;
    cv::Mat depth;
    data.uncompressData(&rgb, &depth, nullptr);
    pcl::IndicesPtr valid_indices(new std::vector<int>);
    auto cloud = rtabmap::util3d::cloudRGBFromSensorData(
      data, depth_decimation, static_cast<float>(depth_max),
      static_cast<float>(depth_min), valid_indices.get());
    std::size_t finite_points = 0;
    for (const auto & point : cloud->points) {
      finite_points += finitePoint(point) ? 1U : 0U;
    }
    const bool ok = !rgb.empty() && !depth.empty() && finite_points > 0;
    passed += ok ? 1 : 0;
    std::cout << "PROBE_NODE id=" << node_message.id
              << " rgb_compressed=" << rgb_compressed
              << " depth_compressed=" << depth_compressed
              << " decompressed=" << (!rgb.empty() && !depth.empty() ? "true" : "false")
              << " rgb=" << rgb.cols << "x" << rgb.rows << " type=" << rgb.type()
              << " depth=" << depth.cols << "x" << depth.rows << " type=" << depth.type()
              << " cloud_points=" << cloud->size()
              << " valid_xyzrgb=" << finite_points
              << " result=" << (ok ? "PASS" : "FAIL") << std::endl;
  }

  const bool success = tested >= required_nodes && passed == tested;
  std::cout << "PROBE_SUMMARY required=" << required_nodes << " tested=" << tested
            << " passed=" << passed << " result=" << (success ? "PASS" : "FAIL")
            << std::endl;
  rclcpp::shutdown();
  return success ? 0 : 5;
}
