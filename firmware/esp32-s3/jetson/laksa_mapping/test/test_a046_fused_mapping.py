import ast
from pathlib import Path
import unittest

from laksa_mapping.fused_policy import FUSED_MAPPING, MAPPING_SOURCES, fused_ready, output_map_topic


ROOT = Path(__file__).resolve().parents[1]
DASHBOARD = ROOT.parent / "laksa_dashboard"
NAV2_CONFIG = ROOT.parent / "laksa_bringup/config/nav2_ackermann.yaml"


class A046FusedMappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = (ROOT / "launch/mapping_stack.launch.py").read_text(encoding="utf-8")
        cls.manager = (ROOT / "laksa_mapping/session_manager.py").read_text(encoding="utf-8")
        cls.rtab = (ROOT / "config/rtabmap_fused.yaml").read_text(encoding="utf-8")
        cls.costmaps = NAV2_CONFIG.read_text(encoding="utf-8")
        cls.setup = (ROOT / "setup.py").read_text(encoding="utf-8")
        cls.cockpit = (DASHBOARD / "laksa_dashboard/cockpit_server.py").read_text(encoding="utf-8")
        cls.ui = (DASHBOARD / "web/index.html").read_text(encoding="utf-8")
        cls.dense_cmake = (ROOT.parent / "laksa_dense_map/CMakeLists.txt").read_text(encoding="utf-8")
        cls.dense_node = (ROOT.parent / "laksa_dense_map/src/dense_map_assembler.cpp").read_text(encoding="utf-8")

    def test_one_runtime_mapping_source_and_one_rtab(self):
        self.assertEqual(MAPPING_SOURCES, (FUSED_MAPPING,))
        self.assertEqual(output_map_topic(), "/map")
        self.assertEqual(self.launch.count('package="rtabmap_slam"'), 2)
        self.assertEqual(self.launch.count('plugin="rtabmap_slam::CoreWrapper"'), 1)
        self.assertEqual(self.launch.count('zed_camera.launch.py'), 1)
        self.assertNotIn("mapping_ab.launch.py", self.setup)
        self.assertNotIn("mapping_shadow.launch.py", self.setup)
        self.assertNotIn("map_ab_session_manager =", self.setup)

    def test_native_rtab_rgbd_scan_icp_contract(self):
        for setting in ("subscribe_rgbd: true", "subscribe_scan: true", "Grid/Sensor: '2'",
                        "Reg/Strategy: '2'", "RGBD/NeighborLinkRefining: 'true'",
                        "RGBD/ProximityBySpace: 'true'"):
            self.assertIn(setting, self.rtab)
        self.assertNotIn("Reg/Force3DoF", self.rtab)
        self.assertIn('("scan", "/laksa/lidar/scan_validated")', self.launch)
        self.assertIn('("rgb/camera_info", "/zed/zed_node/rgb/color/rect/camera_info")', self.launch)
        self.assertNotIn("/scan_raw", self.launch + self.manager)

    def test_rgbd_sync_uses_approximate_source_timestamps(self):
        self.assertIn('"approx_sync": True', self.launch)
        self.assertIn('"approx_sync_max_interval": 0.05', self.launch)
        self.assertIn("sync_queue_size: 10", self.rtab)
        self.assertNotIn("topic_queue_size:", self.rtab)

    def test_composition_is_opt_in_and_uses_installed_plugins(self):
        self.assertIn('DeclareLaunchArgument("compose_rgbd_rtab", default_value="false")', self.launch)
        self.assertIn('plugin="rtabmap_sync::RGBDSync"', self.launch)
        self.assertIn('plugin="rtabmap_slam::CoreWrapper"', self.launch)
        self.assertIn('executable="component_container_mt"', self.launch)
        self.assertEqual(self.launch.count('extra_arguments=[{"use_intra_process_comms": True}]'), 2)
        self.assertIn("condition=UnlessCondition(compose_rgbd_rtab)", self.launch)
        self.assertIn("condition=IfCondition(compose_rgbd_rtab)", self.launch)
        self.assertIn('self.declare_parameter("compose_rgbd_rtab", False)', self.manager)
        self.assertIn('self.declare_parameter("grid_noise_filtering_radius_override", -1.0)', self.manager)
        self.assertIn('self.declare_parameter("cloud_output_voxelized_override", -1)', self.manager)
        self.assertIn("radius_override not in (-1.0, 0.0, 0.05)", self.manager)
        self.assertIn('"grid_noise_filtering_radius": self._grid_noise_filtering_radius', self.manager)

    def test_a046_5_noise_filter_challenger_changes_one_grid_variable(self):
        self.assertIn("Grid/NoiseFilteringRadius: '0.0'", self.rtab)
        self.assertIn("Grid/NoiseFilteringMinNeighbors: '5'", self.rtab)
        self.assertIn("Grid/CellSize: '0.05'", self.rtab)
        self.assertIn("Grid/Sensor: '2'", self.rtab)
        self.assertIn("re.subn(", self.manager)
        self.assertIn("Grid/NoiseFilteringRadius override", self.manager)

    def test_a046_6_cloud_density_challenger_changes_only_output_voxelization(self):
        self.assertIn("cloud_output_voxelized: true", self.rtab)
        self.assertIn("cloud_output_voxelized_override", self.manager)
        self.assertIn("cloud_voxel_override not in (-1, 0, 1)", self.manager)
        self.assertIn("Could not apply cloud_output_voxelized override", self.manager)
        self.assertIn(r"(?:true|false)[ \t]*$", self.manager)
        self.assertNotIn(r"(?:true|false)\s*$", self.manager)
        for unchanged in ("Grid/DepthDecimation: '2'", "Grid/CellSize: '0.05'", "cloud_decimation: 4", "cloud_voxel_size: 0.05"):
            self.assertIn(unchanged, self.rtab)

    def test_legacy_and_composed_topologies_share_parameters_and_remaps(self):
        self.assertIn("parameters=sync_parameters, remappings=sync_remappings", self.launch)
        self.assertIn("parameters=rtab_parameters, remappings=rtab_remappings", self.launch)
        self.assertEqual(self.launch.count('("rgb/image", "/zed/zed_node/rgb/color/rect/image")'), 1)
        self.assertEqual(self.launch.count('("rgb/camera_info", "/zed/zed_node/rgb/color/rect/camera_info")'), 1)
        self.assertEqual(self.launch.count('("depth/image", "/zed/zed_node/depth/depth_registered")'), 1)
        # One RTAB input plus one opt-in dense-cloud generator consume the
        # canonical synchronized RGB-D topic. Both RTAB launch forms share
        # ``rtab_remappings`` rather than duplicating this declaration.
        self.assertEqual(self.launch.count('("rgbd_image", "/laksa/fused_mapping/rgbd_image")'), 2)
        for setting in ('"approx_sync": True', '"approx_sync_max_interval": 0.05', '"queue_size": 10'):
            self.assertEqual(self.launch.count(setting), 1)
        self.assertEqual(self.launch.count('"qos": 2'), 3)

    def test_canonical_topics_and_tf_authorities(self):
        self.assertIn('("odom", "/laksa/odometry/fused")', self.launch)
        self.assertIn('("map", "/map")', self.launch)
        self.assertIn("publish_tf: true", (ROOT / "config/rtabmap_fused.yaml").read_text())
        self.assertEqual(self.launch.count('package="robot_localization"'), 1)
        self.assertIn('name="ekf_filter_node"', self.launch)
        self.assertNotIn("static_transform_publisher", self.launch)
        self.assertNotIn("mapping_shadow_map", self.launch + self.manager)

    def test_fusion_fails_closed_without_good_fresh_scan(self):
        self.assertTrue(fused_ready("GOOD", 0.2, 1.0))
        self.assertFalse(fused_ready("BAD", 0.2, 1.0))
        self.assertFalse(fused_ready("GOOD", None, 1.0))
        self.assertFalse(fused_ready("GOOD", 1.1, 1.0))

    def test_dense_3d_cloud_is_bounded(self):
        zed = (ROOT / "config/indoor_live_zed.yaml").read_text()
        # The production ZED profile does not publish an unbounded raw cloud.
        # Dense output is produced only by the opt-in bounded RTAB utility path.
        self.assertIn("publish_point_cloud: false", zed)
        self.assertIn("cloud_voxel_size: 0.05", self.rtab)
        self.assertIn("cloud_decimation: 4", self.rtab)

    def test_costmap_candidate_uses_all_requested_sources(self):
        self.assertIn("plugins: [lidar_obstacle_layer, zed_voxel_layer, inflation_layer]", self.costmaps)
        self.assertIn("topic: /laksa/lidar/scan_validated", self.costmaps)
        self.assertIn("topic: /zed/zed_node/point_cloud/cloud_registered", self.costmaps)
        self.assertGreaterEqual(self.costmaps.count("marking: true"), 2)
        self.assertGreaterEqual(self.costmaps.count("clearing: true"), 2)
        self.assertIn("plugins: [static_layer, obstacle_layer, inflation_layer]", self.costmaps)
        self.assertIn("global_frame: map", self.costmaps)

    def test_field_lab_has_one_fused_panel_and_no_ab_runtime(self):
        self.assertIn("FUSED MAPPING", self.ui)
        self.assertIn("ONE WORLD MODEL", self.ui)
        self.assertNotIn("MAP A/B LAB", self.ui)
        self.assertNotIn("/api/map-ab/", self.cockpit)
        cockpit_launch = (DASHBOARD / "launch/cockpit.launch.py").read_text()
        self.assertNotIn("map_ab_session_manager", cockpit_launch)
        self.assertIn('OccupancyGrid, "/map"', self.cockpit)
        self.assertIn('PointCloud2, "/laksa/fused_mapping/cloud_map"', self.cockpit)

    def test_a046_6b_dense_map_is_independent_and_opt_in(self):
        self.assertIn('DeclareLaunchArgument("enable_dense_cloud_map", default_value="false")', self.launch)
        self.assertIn('executable="point_cloud_xyzrgb"', self.launch)
        self.assertIn('executable="point_cloud_assembler"', self.launch)
        self.assertEqual(self.launch.count("condition=IfCondition(enable_dense_cloud_map)"), 2)
        self.assertIn('PointCloud2, "/zed/zed_node/point_cloud/cloud_registered"', self.cockpit)
        self.assertIn('"live_rgbd_preview"', self.cockpit)
        self.assertIn('cloud_transform = self._tf_buffer.lookup_transform(', self.cockpit)
        self.assertIn('msg.header.frame_id, rclpy.time.Time()', self.cockpit)
        self.assertIn('output_frame = "map"', self.cockpit)
        self.assertIn('xyz = _transform_xyz(', self.cockpit)
        self.assertEqual(
            self.launch.count('DeclareLaunchArgument("enable_dense_cloud_map", default_value="false")'),
            1,
        )
        self.assertIn('self.declare_parameter("enable_dense_cloud_map", False)', self.manager)
        self.assertIn('"occupancy_cloud":"/laksa/fused_mapping/cloud_map"', self.manager)
        self.assertIn('"dense_cloud":"/zed/zed_node/point_cloud/cloud_registered"', self.manager)

    def test_a046_6b_uses_official_keyframe_and_graph_apis(self):
        for token in ("rtabmap_conversions::mapGraphFromROS", "rtabmap_conversions::nodeFromROS",
                      "rtabmap::util3d::cloudRGBFromSensorData",
                      "rtabmap::util3d::transformPointCloud", "rtabmap::util3d::voxelize"):
            self.assertIn(token, self.dense_node)
        self.assertIn("local_clouds_", self.dense_node)
        self.assertIn("optimized_poses_", self.dense_node)
        self.assertIn("if (!dirty_)", self.dense_node)
        self.assertIn('find_package(RTABMap 0.22.0 REQUIRED COMPONENTS core)', self.dense_cmake)
        self.assertIn('rtabmap::core', self.dense_cmake)

    def test_a046_6b_field_lab_distinguishes_dense_and_occupancy_clouds(self):
        self.assertIn('PointCloud2, "/zed/zed_node/point_cloud/cloud_registered"', self.cockpit)
        self.assertIn('"live_rgbd_preview"', self.cockpit)
        self.assertIn('"occupancy_cloud"', self.cockpit)
        self.assertIn("max_cloud_points", self.cockpit)
        self.assertIn('id="cloudSource"', self.ui)

    def test_a046_6b_does_not_modify_occupancy_resolution_controls(self):
        for unchanged in ("Grid/NoiseFilteringRadius: '0.0'", "Grid/DepthDecimation: '2'",
                          "Grid/CellSize: '0.05'", "Grid/Sensor: '2'"):
            self.assertIn(unchanged, self.rtab)
        for forbidden in ("Grid/DepthDecimation", "Grid/CellSize", "Grid/Sensor",
                          "Grid/NoiseFilteringRadius"):
            self.assertNotIn(forbidden, self.dense_node)

    def test_release_export_waits_for_low_rate_rtab_map(self):
        self.assertIn(
            'self.declare_parameter("occupancy_save_timeout_sec", 15.0)',
            self.manager,
        )
        self.assertIn(
            '"-p", f"save_map_timeout:={self._occupancy_save_timeout}"',
            self.manager,
        )
        self.assertIn("timeout=self._occupancy_save_timeout + 10.0", self.manager)

    def test_stationary_detection_snapshots_concurrent_odom_samples(self):
        self.assertIn("with self._lock:\n            self._motion_samples.append(sample)", self.manager)
        self.assertIn("samples = tuple(self._motion_samples)", self.manager)
        self.assertIn("for sample in samples", self.manager)
        self.assertNotIn("for sample in self._motion_samples", self.manager)

    def test_no_motion_publishers(self):
        joined = self.launch + self.manager + self.cockpit
        measurements = (
            ROOT.parent / "laksa_bringup/scripts/state_measurements_node.py"
        ).read_text(encoding="utf-8")
        for forbidden in ("cmd_vel", "AckermannDrive", "DriveCommand", "motor_command"):
            self.assertNotIn(forbidden, joined)
            self.assertNotIn(forbidden, measurements)
        self.assertIn('Odometry, "/laksa/vesc_odom"', measurements)
        ast.parse(self.manager)
        ast.parse(self.cockpit)


if __name__ == "__main__":
    unittest.main()
