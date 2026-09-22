import ast
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
SERVER = PACKAGE / "laksa_dashboard" / "cockpit_server.py"


class RuntimeInvariantTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SERVER.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def test_planning_preview_keeps_one_action_send_and_auto_heading(self):
        self.assertEqual(self.source.count("self._planner_client.send_goal_async"), 1)
        self.assertEqual(self.source.count("self._follow_client.send_goal_async"), 1)
        self.assertIn('"heading_mode": "AUTO_HEADING"', self.source)
        self.assertNotIn("BEST_APPROACH", self.source)

    def test_safe_mask_computation_is_not_in_planning_grid_callback(self):
        functions = {
            node.name: node
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        callback_calls = {
            node.func.id
            for node in ast.walk(functions["_planning_grid_cb"])
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        worker_calls = {
            node.func.id
            for node in ast.walk(functions["_compute_safe_mask"])
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertNotIn("encoded_mask", callback_calls)
        self.assertIn("encoded_mask", worker_calls)
        self.assertIn("max_workers=1", self.source)

    def test_pose_and_safe_mask_payloads_do_not_duplicate_bulk_state(self):
        dictionaries = [node for node in ast.walk(self.tree) if isinstance(node, ast.Dict)]
        typed = {}
        for node in dictionaries:
            literal = {}
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(key.value, str):
                    literal[key.value] = value
            message_type = literal.get("type")
            if isinstance(message_type, ast.Constant) and isinstance(message_type.value, str):
                typed.setdefault(message_type.value, []).append(set(literal))
        self.assertTrue(any("trajectory" not in keys for keys in typed["pose"]))
        self.assertTrue(any("mask" in keys and "data" not in keys for keys in typed["safe_goal_mask"]))

    def test_browser_pose_latency_is_persisted_for_qualification(self):
        self.assertIn('app.router.add_post("/api/telemetry/pose-latency"', self.source)
        self.assertIn('self._latest["field_lab_latency"] = validated', self.source)


if __name__ == "__main__":
    unittest.main()
