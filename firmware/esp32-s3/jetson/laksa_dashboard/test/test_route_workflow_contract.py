from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RouteWorkflowContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = (ROOT / "laksa_dashboard" / "cockpit_server.py").read_text()
        cls.app = (ROOT / "web" / "app.js").read_text()
        cls.html = (ROOT / "web" / "index.html").read_text()

    def test_continuous_mask_is_rendered_without_goal_candidate_dots(self):
        self.assertIn("safeRaster", self.app)
        self.assertIn("drawRaster(ctx,map2d.safeImage", self.app)
        self.assertNotIn("goalCandidates", self.app)
        self.assertNotIn("candidateDots", self.app)

    def test_safe_click_uses_exact_world_coordinate_and_never_snaps(self):
        self.assertIn("return maskContains", self.app)
        self.assertIn("JSON.stringify({x:goal.x,y:goal.y", self.app)
        self.assertNotIn("nearestCandidate", self.app)
        self.assertNotIn("snapGoal", self.app)

    def test_invalid_click_never_calls_planner(self):
        self.assertIn("if(!goal){if(!map2d.safe||!map2d.safe.safe_cells)", self.app)
        self.assertIn("else localPlanningState('INVALID_GOAL','Choose a highlighted safe area.')", self.app)
        self.assertIn("Choose a highlighted safe area.", self.app)

    def test_exact_planner_path_is_retained_for_followpath(self):
        self.assertIn("self._planned_path = path", self.server)
        self.assertIn("exact_path = self._planned_path", self.server)
        self.assertIn("action_goal.path = exact_path", self.server)
        self.assertIn('ActionClient(self, FollowPath, "/follow_path")', self.server)
        self.assertIn('self._controller_state_client = self._nav_state_clients["controller_server"]', self.server)
        self.assertIn("self._controller_active", self.server)
        self.assertIn('action_goal.controller_id = "FollowPath"', self.server)

    def test_follow_route_uses_product_confirmation_and_internal_arm(self):
        self.assertIn('id="followRoute"', self.html)
        self.assertIn('body.get("confirmed") is not True', self.server)
        self.assertIn('SetBool, "/laksa/autonomy/set_armed"', self.server)
        self.assertIn("START AUTONOMOUS ROUTE?", self.html)
        self.assertIn("Maximum speed: 0.15 m/s", self.html)
        self.assertIn("Xbox input immediately takes manual control", self.html)
        self.assertIn('id="startRoute"', self.html)
        self.assertNotIn("confirm(", self.app)
        self.assertNotIn("explicit ARM", self.html)
        for forbidden in ("/laksa/command", "/laksa/brake", "VESC", "PCA9685"):
            self.assertNotIn(f'create_publisher({forbidden}', self.server)

    def test_follow_failure_cancel_and_completion_request_disarm(self):
        self.assertIn('self._request_disarm("FollowPath finished")', self.server)
        self.assertIn('self._request_disarm("operator canceled FollowPath")', self.server)
        self.assertIn("Mapping has a fatal health state; movement remains inhibited", self.server)
        self.assertIn("Route preflight failed:", self.server)
        self.assertIn('self._autonomy_health != "READY"', self.server)
        self.assertIn('int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED', self.server)
        self.assertIn('state = "ROUTE_COMPLETE"', self.server)

    def test_planner_uses_the_exact_canonical_ui_pose(self):
        self.assertIn('lookup_transform(\n                "map", "base_footprint"', self.server)
        self.assertIn("action_goal.start = start", self.server)
        self.assertIn("action_goal.use_start = True", self.server)

    def test_follow_waits_for_acceptance_and_has_full_preflight(self):
        self.assertIn("while not future.done()", self.server)
        self.assertIn("if not goal_handle.accepted", self.server)
        self.assertLess(
            self.server.index('if not goal_handle.accepted', self.server.index('async def _api_follow_path')),
            self.server.index('"FOLLOWING"', self.server.index('async def _api_follow_path')),
        )
        for gate in (
            "SmacPlannerHybrid is not ACTIVE", "Local obstacle costmap is stale",
            "Xbox manual-control safety channel is not ONLINE",
            "ESP32 command link is not ONLINE", "VESC telemetry is stale or faulted",
            "Stop the vehicle before starting the route",
        ):
            self.assertIn(gate, self.server)

    def test_safe_overlay_requires_exact_map_geometry(self):
        self.assertIn('"visible": True', self.server)
        self.assertIn("sameGridGeometry(map2d.grid,m)", self.app)

    def test_live_mapping_starts_one_official_nav2_stack_without_amcl(self):
        launch = (ROOT / "launch" / "planning_preview.launch.py").read_text()
        self.assertIn('package="nav2_planner"', launch)
        self.assertIn('package="nav2_controller"', launch)
        self.assertNotIn('package="nav2_map_server"', launch)
        self.assertNotIn('package="nav2_amcl"', launch)
        self.assertIn('"planning_preview.launch.py"', self.server)
        self.assertIn('"LIVE_MAPPING_READY"', self.server)
        self.assertIn('localization_source="RTAB"', self.server)
        self.assertIn('navigation.get("state") not in ("READY", "LIVE_MAPPING_READY")', self.server)
        self.assertIn("self._planner_active = True", self.server)
        self.assertIn("self._controller_active = True", self.server)

    def test_live_map_route_is_revalidated_before_internal_authority(self):
        follow = self.server.split("    def _follow_preflight_reason", 1)[1].split(
            "    def _poll_planner_state", 1
        )[0]
        self.assertIn("for stamped in path.poses", follow)
        self.assertIn("_candidate_footprint_is_free", follow)
        self.assertIn("live map changed", follow)

    def test_transient_dds_discovery_does_not_erase_validated_lifecycle_state(self):
        self.assertIn("if process is None or process.poll() is not None:\n                self._planner_active = False", self.server)
        self.assertIn("if process is None or process.poll() is not None:\n                self._controller_active = False", self.server)
        planner_done = self.server.split("    def _planner_state_done", 1)[1].split(
            "    def _poll_controller_state", 1
        )[0]
        controller_done = self.server.split("    def _controller_state_done", 1)[1].split(
            "    def _clear_preview", 1
        )[0]
        self.assertIn("except Exception:\n            return", planner_done)
        self.assertIn("except Exception:\n            return", controller_done)

    def test_transient_live_status_keeps_exact_preview_but_follow_fails_closed(self):
        tick = self.server.split("    def _planning_tick", 1)[1].split(
            "    def _set_planning_state", 1
        )[0]
        self.assertIn('(\"PLANNING\", \"PATH_READY\", \"STARTING_ROUTE\", \"FOLLOWING\")', tick)
        self.assertIn("follow_available = self._follow_available()", tick)
        self.assertNotIn("Preview invalidated because its map or pose is no longer current", tick)

    def test_empty_safe_mask_has_a_concrete_reason(self):
        self.assertIn('"reason": "" if result["safe_cells"]', self.server)
        self.assertIn("WAITING_FOR_SAFE_AREA", self.server)
        self.assertIn("Map still initializing; no collision-free goal area", self.app)

    def test_pose_uses_dedicated_latest_value_transport(self):
        self.assertIn('app.router.add_get("/ws/pose", self._pose_ws)', self.server)
        self.assertIn('Odometry, "/laksa/odometry/fused", self._fused_odom_cb', self.server)
        self.assertIn("callback_group=self._pose_callback_group", self.server)
        self.assertIn("self._publish_pose(sample)", self.server)
        publish_pose = self.server.split("    def _publish_pose", 1)[1].split("    def _performance_tick", 1)[0]
        self.assertNotIn('lookup_transform("odom", "base_footprint"', publish_pose)
        self.assertNotIn("create_timer(0.05", self.server)
        self.assertIn("def _update_cloud_subscriptions", self.server)
        self.assertIn('mapping.get("state") in ("STARTING", "MAPPING", "DEGRADED")', self.server)
        self.assertIn("self.destroy_subscription(self._zed_cloud_subscription)", self.server)
        self.assertIn("self.destroy_subscription(self._global_cloud_subscription)", self.server)
        self.assertIn("function connectPose()", self.app)
        render = self.app.split("function renderFrame(){", 1)[1].split("}\n", 1)[0]
        self.assertLess(render.index("mailbox.take('pose')"), render.index("mailbox.take('cloud')"))

    def test_hero_journey_requires_a_valid_route(self):
        self.assertIn("heroJourneyAvailable(state.planning)", self.app)
        self.assertIn("heroJourney').disabled=!hero", self.app)


if __name__ == "__main__":
    unittest.main()
