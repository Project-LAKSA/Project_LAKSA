#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace
{
diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

class DenseMapAssembler final : public rclcpp::Node
{
public:
  DenseMapAssembler()
  : Node("dense_map_assembler"), dirty_(false), initial_map_requested_(false),
    local_points_(0), output_points_(0), rebuild_ms_(0.0), published_messages_(0)
  {
    map_data_topic_ = declare_parameter<std::string>("map_data_topic", "/laksa/fused_mapping/mapData");
    map_data_service_ = declare_parameter<std::string>("map_data_service", "/laksa/fused_mapping/rtabmap/get_map_data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/laksa/fused_mapping/dense_cloud_map");
    depth_decimation_ = declare_parameter<int>("depth_decimation", 4);
    depth_min_ = declare_parameter<double>("depth_min", 0.3);
    depth_max_ = declare_parameter<double>("depth_max", 8.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.02);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    max_cached_nodes_ = declare_parameter<int>("max_cached_nodes", 250);
    max_cached_points_ = declare_parameter<int>("max_cached_points", 4000000);

    if (depth_decimation_ < 1 || depth_min_ < 0.0 || depth_max_ <= depth_min_ ||
      voxel_size_ < 0.0 || publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 1.0 ||
      max_cached_nodes_ < 1 || max_cached_points_ < 1000)
    {
      throw std::invalid_argument("Invalid dense-map limits or RGB-D filtering parameters");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    map_client_ = create_client<rtabmap_msgs::srv::GetMap>(map_data_service_);
    map_subscription_ = create_subscription<rtabmap_msgs::msg::MapData>(
      map_data_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&DenseMapAssembler::mapDataCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DenseMapAssembler::publishIfChanged, this));
    bootstrap_timer_ = create_wall_timer(5s, std::bind(&DenseMapAssembler::requestInitialMap, this));

    RCLCPP_INFO(
      get_logger(),
      "Dense RGB-D map: %s -> %s (decimation=%d depth=%.2f..%.2f m voxel=%.3f m rate<=%.2f Hz nodes<=%d points<=%d)",
      map_data_topic_.c_str(), output_topic_.c_str(), depth_decimation_, depth_min_, depth_max_,
      voxel_size_, publish_rate_hz_, max_cached_nodes_, max_cached_points_);
  }

private:
  using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;

  void requestInitialMap()
  {
    bootstrap_timer_->cancel();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (initial_map_requested_ || !local_clouds_.empty()) {
        return;
      }
      initial_map_requested_ = true;
    }
    if (!map_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Map-data bootstrap service unavailable; live MapData remains active");
      return;
    }
    auto request = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
    request->global_map = false;
    request->optimized = true;
    request->graph_only = false;
    map_client_->async_send_request(
      request,
      [this](rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedFuture response) {
        processMapData(response.get()->data);
      });
  }

  void mapDataCallback(const rtabmap_msgs::msg::MapData::ConstSharedPtr message)
  {
    processMapData(*message);
  }

  void processMapData(const rtabmap_msgs::msg::MapData & message)
  {
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    rtabmap::Transform map_to_odom;
    rtabmap_conversions::mapGraphFromROS(message.graph, poses, links, map_to_odom);

    bool changed = false;
    size_t skipped_existing = 0;
    size_t skipped_compressed = 0;
    size_t skipped_models = 0;
    size_t skipped_decompressed = 0;
    size_t skipped_empty_cloud = 0;
    size_t added_clouds = 0;
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id_ = message.header.frame_id.empty() ? "map" : message.header.frame_id;
    stamp_ = message.header.stamp;

    if (poses != optimized_poses_) {
      optimized_poses_ = std::move(poses);
      changed = true;
    }

    for (const auto & node_message : message.nodes) {
      const int id = node_message.id;
      if (id <= 0 || local_clouds_.find(id) != local_clouds_.end()) {
        ++skipped_existing;
        continue;
      }
      rtabmap::Signature signature = rtabmap_conversions::nodeFromROS(node_message);
      auto & data = signature.sensorData();
      if (data.imageCompressed().empty() || data.depthOrRightCompressed().empty()) {
        ++skipped_compressed;
        continue;
      }
      if (data.cameraModels().empty() && data.stereoCameraModels().empty()) {
        ++skipped_models;
        continue;
      }

      cv::Mat image;
      cv::Mat depth;
      data.uncompressData(&image, &depth, nullptr);
      if (data.imageRaw().empty() || data.depthOrRightRaw().empty()) {
        ++skipped_decompressed;
        continue;
      }

      pcl::IndicesPtr valid_indices(new std::vector<int>);
      Cloud::Ptr cloud = rtabmap::util3d::cloudRGBFromSensorData(
        data, depth_decimation_, static_cast<float>(depth_max_),
        static_cast<float>(depth_min_), valid_indices.get());
      if (cloud->empty()) {
        ++skipped_empty_cloud;
        continue;
      }
      if (voxel_size_ > 0.0) {
        cloud = rtabmap::util3d::voxelize(
          cloud, valid_indices, static_cast<float>(voxel_size_));
      }
      Cloud::Ptr finite(new Cloud);
      std::vector<int> retained;
      pcl::removeNaNFromPointCloud(*cloud, *finite, retained);
      if (finite->empty()) {
        ++skipped_empty_cloud;
        continue;
      }
      finite->is_dense = true;
      local_points_ += finite->size();
      local_clouds_.emplace(id, std::move(finite));
      ++added_clouds;
      changed = true;
    }

    while (static_cast<int>(local_clouds_.size()) > max_cached_nodes_ ||
      static_cast<int64_t>(local_points_) > max_cached_points_)
    {
      auto oldest = local_clouds_.begin();
      local_points_ -= oldest->second->size();
      local_clouds_.erase(oldest);
    }
    dirty_ = dirty_ || changed;
    if (!message.nodes.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "MapData graph=%zu nodes=%zu added=%zu cached=%zu points=%zu skip(existing=%zu compressed=%zu models=%zu decompressed=%zu empty=%zu)",
        optimized_poses_.size(), message.nodes.size(), added_clouds, local_clouds_.size(),
        local_points_, skipped_existing, skipped_compressed, skipped_models,
        skipped_decompressed, skipped_empty_cloud);
    }
  }

  void publishIfChanged()
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    std::map<int, Cloud::Ptr> clouds;
    std::map<int, rtabmap::Transform> poses;
    std::string frame;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!dirty_) {
        return;
      }
      dirty_ = false;
      clouds = local_clouds_;
      poses = optimized_poses_;
      frame = frame_id_;
      stamp = stamp_;
    }

    const auto started = std::chrono::steady_clock::now();
    Cloud::Ptr assembled(new Cloud);
    size_t reserve = 0;
    for (const auto & item : clouds) {
      if (poses.find(item.first) != poses.end()) {
        reserve += item.second->size();
      }
    }
    assembled->reserve(reserve);
    size_t matched_nodes = 0;
    for (const auto & item : clouds) {
      const auto pose = poses.find(item.first);
      if (pose == poses.end() || pose->second.isNull()) {
        continue;
      }
      ++matched_nodes;
      Cloud::Ptr transformed = rtabmap::util3d::transformPointCloud(item.second, pose->second);
      *assembled += *transformed;
    }
    if (assembled->empty()) {
      RCLCPP_WARN(
        get_logger(), "Dense rebuild empty: cached_nodes=%zu graph_nodes=%zu matched_nodes=%zu",
        clouds.size(), poses.size(), matched_nodes);
      return;
    }
    if (voxel_size_ > 0.0) {
      assembled = rtabmap::util3d::voxelize(assembled, static_cast<float>(voxel_size_));
    }
    assembled->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*assembled, output);
    output.header.frame_id = frame;
    output.header.stamp = stamp;
    publisher_->publish(output);

    output_points_ = assembled->size();
    rebuild_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    ++published_messages_;
    publishDiagnostics(output.data.size(), clouds.size(), poses.size());
  }

  void publishDiagnostics(size_t bytes, size_t cached_nodes, size_t graph_nodes)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "laksa_mapping/dense_rgbd_map";
    status.hardware_id = "rtabmap-keyframes";
    status.message = "DENSE_MAP_READY";
    status.values = {
      keyValue("cached_nodes", std::to_string(cached_nodes)),
      keyValue("graph_nodes", std::to_string(graph_nodes)),
      keyValue("cached_local_points", std::to_string(local_points_)),
      keyValue("output_points", std::to_string(output_points_)),
      keyValue("serialized_bytes", std::to_string(bytes)),
      keyValue("rebuild_ms", std::to_string(rebuild_ms_)),
      keyValue("published_messages", std::to_string(published_messages_)),
      keyValue("depth_decimation", std::to_string(depth_decimation_)),
      keyValue("depth_min_m", std::to_string(depth_min_)),
      keyValue("depth_max_m", std::to_string(depth_max_)),
      keyValue("voxel_size_m", std::to_string(voxel_size_))};
    array.status.push_back(std::move(status));
    diagnostics_->publish(std::move(array));
  }

  std::string map_data_topic_;
  std::string map_data_service_;
  std::string output_topic_;
  int depth_decimation_;
  double depth_min_;
  double depth_max_;
  double voxel_size_;
  double publish_rate_hz_;
  int max_cached_nodes_;
  int max_cached_points_;

  std::mutex mutex_;
  std::map<int, Cloud::Ptr> local_clouds_;
  std::map<int, rtabmap::Transform> optimized_poses_;
  std::string frame_id_ = "map";
  builtin_interfaces::msg::Time stamp_;
  bool dirty_;
  bool initial_map_requested_;
  size_t local_points_;
  size_t output_points_;
  double rebuild_ms_;
  uint64_t published_messages_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr map_subscription_;
  rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedPtr map_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr bootstrap_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DenseMapAssembler>());
  rclcpp::shutdown();
  return 0;
}
