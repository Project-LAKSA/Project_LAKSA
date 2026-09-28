#!/usr/bin/env python3
"""Make the pinned MPPI PathAlignCritic test validate per-trajectory semantics.

The original aggregate float32 reduction is architecture-order-sensitive on
AArch64. This patch changes only the pinned upstream test: every trajectory
must still have exactly the expected 6.6f cost.
"""

import argparse
from pathlib import Path


def patch_test(path: Path) -> None:
    text = path.read_text()
    original = "  EXPECT_NEAR(xt::sum(costs, immediate)(), 6600.0, 1e-2);"
    replacement = """  for (const auto cost : costs) {
    EXPECT_FLOAT_EQ(cost, 6.6f);
  }"""
    if replacement in text:
        return
    if text.count(original) != 1:
        raise SystemExit("unexpected pinned Nav2 MPPI PathAlignCritic test layout")
    path.write_text(text.replace(original, replacement, 1))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Patch the pinned Nav2 MPPI PathAlignCritic test semantics."
    )
    parser.add_argument("test_source", type=Path)
    args = parser.parse_args()
    patch_test(args.test_source)


if __name__ == "__main__":
    main()
