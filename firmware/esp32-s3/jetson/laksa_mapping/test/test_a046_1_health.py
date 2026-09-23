from pathlib import Path
import unittest

from laksa_mapping.health_model import HealthThresholds, activity_state, derive_fused_health


ROOT = Path(__file__).resolve().parents[1]
READY_AGES = {
    "rgb": 0.1, "depth": 0.1, "rgbd": 0.3, "fused_odom": 0.05,
    "scan": 0.05, "rtab_processed": 0.4,
    "map_content": 12.0, "cloud_content": 12.0,
}
TF_OK = {"map_to_odom": {"available": True}, "odom_to_base": {"available": True}}


def health(**changes):
    values = dict(active=True, startup=False, elapsed_sec=20.0, ages=dict(READY_AGES),
                  lidar_health="GOOD", tf_health=TF_OK, process_alive=True,
                  map_available=True, cloud_available=True, stationary=True)
    values.update(changes)
    return derive_fused_health(**values)


class A0461HealthTest(unittest.TestCase):
    def test_stationary_healthy_pipeline_is_ready_and_quiescent(self):
        result = health()
        self.assertEqual(result["fusion_state"], "FUSED_READY")
        self.assertEqual(result["reason_code"], "PIPELINE_HEALTHY_MAP_QUIESCENT")
        self.assertEqual(result["map_activity"], "QUIESCENT")
        self.assertEqual(result["cloud_activity"], "QUIESCENT")

    def test_old_output_publication_does_not_determine_health(self):
        result = health(ages=READY_AGES | {"map_content": 200.0, "cloud_content": 300.0})
        self.assertEqual(result["fusion_state"], "FUSED_READY")

    def test_fresh_content_is_updating(self):
        result = health(ages=READY_AGES | {"map_content": 1.0, "cloud_content": 2.0})
        self.assertEqual(result["map_activity"], "UPDATING")
        self.assertEqual(result["cloud_activity"], "UPDATING")
        self.assertEqual(result["reason_code"], "PIPELINE_HEALTHY_UPDATING")

    def test_stale_required_inputs_have_deterministic_reasons(self):
        cases = (("rgbd", "RGBD_STALE"), ("fused_odom", "ODOMETRY_STALE"), ("scan", "LIDAR_STALE"))
        for key, reason in cases:
            with self.subTest(key=key):
                result = health(ages=READY_AGES | {key: 7.0})
                self.assertEqual(result["fusion_state"], "FUSED_DEGRADED")
                self.assertEqual(result["reason_code"], reason)

    def test_bad_lidar_missing_tf_and_rtab_stall(self):
        self.assertEqual(health(lidar_health="BAD")["reason_code"], "LIDAR_BAD")
        self.assertEqual(health(tf_health={})["reason_code"], "TF_UNAVAILABLE")
        stalled = health(ages=READY_AGES | {"rtab_processed": 8.0})
        self.assertEqual((stalled["fusion_state"], stalled["reason_code"]),
                         ("FUSED_DEGRADED", "RTAB_PROCESSING_TIMEOUT"))

    def test_startup_grace_then_missing_map_error(self):
        grace = health(startup=True, elapsed_sec=10.0, map_available=False, cloud_available=False)
        self.assertEqual(grace["reason_code"], "STARTUP_GRACE")
        expired = health(elapsed_sec=50.0, map_available=False)
        self.assertEqual((expired["fusion_state"], expired["reason_code"]),
                         ("FUSED_ERROR", "MAP_NOT_PRODUCED"))

    def test_moving_without_progress_is_degraded(self):
        result = health(stationary=False)
        self.assertEqual(result["map_activity"], "STALE")
        self.assertEqual(result["fusion_state"], "FUSED_DEGRADED")
        self.assertEqual(result["reason_code"], "MOVING_WITHOUT_MAPPING_PROGRESS")

    def test_process_stop_is_error_and_unavailable_is_explicit(self):
        self.assertEqual(health(process_alive=False)["reason_code"], "RTAB_PROCESS_STOPPED")
        self.assertEqual(activity_state(available=False, content_age_sec=None,
                         pipeline_healthy=True, stationary=True, threshold_sec=5), "UNAVAILABLE")

    def test_stationary_output_quiescence_does_not_mask_callback_liveness(self):
        no_new_map_nodes = READY_AGES | {"map_content": 1800.0, "cloud_content": 1800.0}
        result = health(ages=no_new_map_nodes, stationary=True)
        self.assertEqual(result["fusion_state"], "FUSED_READY")
        self.assertEqual(result["rtab_state"], "PROCESSING")
        self.assertEqual(result["reason_code"], "PIPELINE_HEALTHY_MAP_QUIESCENT")

    def test_moving_with_fresh_inputs_and_stopped_rtab_is_fatal(self):
        result = health(ages=READY_AGES | {"rtab_processed": 16.0}, stationary=False)
        self.assertEqual((result["fusion_state"], result["reason_code"]),
                         ("FUSED_ERROR", "RTAB_PROCESSING_TIMEOUT"))

    def test_stationary_stale_rgbd_is_not_benign_quiescence(self):
        result = health(ages=READY_AGES | {"rgbd": 16.0}, stationary=True)
        self.assertEqual((result["fusion_state"], result["reason_code"]),
                         ("FUSED_ERROR", "RGBD_STALE"))

    def test_dead_rtab_is_fatal_while_stationary(self):
        result = health(process_alive=False, stationary=True)
        self.assertEqual((result["fusion_state"], result["reason_code"]),
                         ("FUSED_ERROR", "RTAB_PROCESS_STOPPED"))

    def test_processing_resume_after_output_quiescence_is_clean(self):
        stopped = health(ages=READY_AGES | {"rtab_processed": 8.0}, stationary=True)
        resumed = health(ages=READY_AGES | {"map_content": 1800.0, "cloud_content": 1800.0},
                         stationary=True)
        self.assertEqual(stopped["reason_code"], "RTAB_PROCESSING_TIMEOUT")
        self.assertEqual((resumed["fusion_state"], resumed["reason_code"]),
                         ("FUSED_READY", "PIPELINE_HEALTHY_MAP_QUIESCENT"))

    def test_shutdown_calls_are_idempotent(self):
        adapter = (ROOT / "laksa_mapping/zed_base_pose_adapter.py").read_text()
        manager = (ROOT / "laksa_mapping/session_manager.py").read_text()
        measurements = (ROOT.parent / "laksa_bringup/scripts/state_measurements_node.py").read_text()
        for source in (adapter, manager, measurements):
            self.assertIn("if rclpy.ok():", source)

    def test_single_mapper_no_shadow_or_motion_runtime(self):
        launch = (ROOT / "launch/mapping_stack.launch.py").read_text()
        dashboard = (ROOT.parent / "laksa_dashboard/laksa_dashboard/cockpit_server.py").read_text()
        measurements = (ROOT.parent / "laksa_bringup/scripts/state_measurements_node.py").read_text()
        ui = (ROOT.parent / "laksa_dashboard/web/index.html").read_text()
        self.assertEqual(launch.count('package="rtabmap_slam"'), 2)
        self.assertEqual(launch.count('plugin="rtabmap_slam::CoreWrapper"'), 1)
        self.assertIn("condition=UnlessCondition(compose_rgbd_rtab)", launch)
        self.assertIn("condition=IfCondition(compose_rgbd_rtab)", launch)
        self.assertNotIn("mapping_shadow", launch + dashboard)
        for value in ("SIDE BY SIDE", "DIFFERENCE OVERLAY", "A/B verdict"):
            self.assertNotIn(value, ui)
        for value in ("cmd_vel", "AckermannDrive", "DriveCommand"):
            self.assertNotIn(value, launch + dashboard)
            self.assertNotIn(value, measurements)
        self.assertIn('executable="state_measurements_node.py"', launch)
        self.assertIn('Odometry, "/laksa/vesc_odom"', measurements)


if __name__ == "__main__":
    unittest.main()
