"""Deterministic, dependency-free staged search with constraint-first ranking."""

from __future__ import annotations

import itertools
import random
from pathlib import Path

import yaml


def _nested_set(target: dict, dotted_key: str, value):
    cursor = target
    parts = dotted_key.split(".")
    for part in parts[:-1]: cursor = cursor.setdefault(part, {})
    cursor[parts[-1]] = value


def candidate_configurations(search_path: Path, count: int | None = None) -> list[dict]:
    spec = yaml.safe_load(search_path.read_text(encoding="utf-8"))
    rng = random.Random(int(spec.get("seed", 2906)))
    hybrid_items = sorted(spec["hybrid"].items())
    smoother_items = sorted(spec["constrained_smoother"].items())
    hybrid_products = list(itertools.product(*(values for _, values in hybrid_items)))
    smoother_products = list(itertools.product(*(values for _, values in smoother_items)))
    target = count or 80
    hybrid_candidates = []
    for values in hybrid_products:
        params = {key: value for (key, _), value in zip(hybrid_items, values)}
        hybrid_candidates.append({"method": "HYBRID_RAW", "planner": params})
    rng.shuffle(hybrid_candidates)
    candidates = hybrid_candidates[:target // 2]
    # Sample the much larger planner+smoother product without materializing it.
    while len(candidates) < target:
        planner_values = rng.choice(hybrid_products)
        smoother_values = tuple(rng.choice(values) for _, values in smoother_items)
        smoother = {}
        for (key, _), value in zip(smoother_items, smoother_values): _nested_set(smoother, key, value)
        item = {"method": "HYBRID_CONSTRAINED",
                "planner": {key: value for (key, _), value in zip(hybrid_items, planner_values)}, "smoother": smoother}
        if item not in candidates: candidates.append(item)
    rng.shuffle(candidates)
    return candidates


def stage_plan(search_path: Path) -> list[dict]:
    spec = yaml.safe_load(search_path.read_text(encoding="utf-8"))
    return [{"name": name, **values} for name, values in spec["stages"].items()]


def select_survivors(store, config_hashes: list[str], count: int) -> list[str]:
    return sorted(config_hashes, key=store.ranking_key)[:count]
