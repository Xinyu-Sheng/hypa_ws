#!/usr/bin/env python3
"""将 zmotion_driver axis 配置从 v1(物理顺序) 迁移到 v2(逻辑顺序)。

v1 语义:
  logical_indices[physical_axis] = logical_axis
  其余 axis.* 数组也按 physical_axis 顺序

v2 语义:
  logical_indices[logical_axis] = physical_axis
  其余 axis.* 数组按 logical_axis 顺序
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Any

import yaml

K_AXIS_COUNT = 12
ARRAY_KEYS = [
    "joint_names",
    "position_modes",
    "zero_offsets",
    "units",
    "directions",
    "speeds",
    "accels",
    "decels",
    "position_topics",
]


def _find_node_config(data: Any) -> tuple[str, dict[str, Any]]:
    if not isinstance(data, dict):
        raise ValueError("YAML 顶层必须是 map")

    for key, value in data.items():
        if not isinstance(value, dict):
            continue
        ros_params = value.get("ros__parameters")
        if isinstance(ros_params, dict) and isinstance(ros_params.get("axis"), dict):
            return key, value

    raise ValueError("未找到包含 ros__parameters.axis 的节点配置")


def _validate_permutation(values: list[int], name: str, n: int) -> None:
    if len(values) != n:
        raise ValueError(f"{name} 长度必须为 {n}，实际为 {len(values)}")
    seen = set()
    for i, v in enumerate(values):
        if not isinstance(v, int):
            raise ValueError(f"{name}[{i}] 不是整数: {v!r}")
        if v < 0 or v >= n:
            raise ValueError(f"{name}[{i}] 越界: {v}，合法范围 [0, {n - 1}]")
        if v in seen:
            raise ValueError(f"{name} 包含重复值: {v}")
        seen.add(v)


def _migrate_axis(axis_cfg: dict[str, Any], axis_count: int) -> dict[str, Any]:
    out = copy.deepcopy(axis_cfg)

    old_logical_indices_raw = out.get("logical_indices")
    if not isinstance(old_logical_indices_raw, list):
        raise ValueError("axis.logical_indices 缺失或不是数组")

    old_logical_indices = [int(v) for v in old_logical_indices_raw]
    _validate_permutation(old_logical_indices, "axis.logical_indices(v1)", axis_count)

    # v1: logical_indices[physical] = logical
    # v2: logical_indices[logical] = physical
    logical_to_physical = [-1] * axis_count
    for physical_axis, logical_axis in enumerate(old_logical_indices):
        logical_to_physical[logical_axis] = physical_axis

    _validate_permutation(logical_to_physical, "axis.logical_indices(v2)", axis_count)

    out["logical_indices"] = logical_to_physical

    # 其余数组从 physical 顺序重排为 logical 顺序
    for key in ARRAY_KEYS:
        raw = out.get(key)
        if raw is None:
            continue
        if not isinstance(raw, list):
            raise ValueError(f"axis.{key} 不是数组")
        if len(raw) != axis_count:
            raise ValueError(
                f"axis.{key} 长度必须为 {axis_count}，实际为 {len(raw)}"
            )

        reordered = [None] * axis_count
        for physical_axis, logical_axis in enumerate(old_logical_indices):
            reordered[logical_axis] = raw[physical_axis]
        out[key] = reordered

    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="迁移 zmotion_driver axis 配置：v1(物理顺序) -> v2(逻辑顺序)"
    )
    parser.add_argument("input", type=Path, help="输入 YAML 文件路径")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="输出 YAML 文件路径；不填则覆盖输入文件",
    )
    parser.add_argument(
        "--axis-count",
        type=int,
        default=K_AXIS_COUNT,
        help=f"轴数量（默认 {K_AXIS_COUNT}）",
    )
    args = parser.parse_args()

    input_path: Path = args.input
    output_path: Path = args.output if args.output is not None else input_path

    with input_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    node_key, node_cfg = _find_node_config(data)
    ros_params = node_cfg["ros__parameters"]
    axis_cfg = ros_params["axis"]

    migrated_axis = _migrate_axis(axis_cfg, args.axis_count)
    ros_params["axis"] = migrated_axis

    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )

    print(f"迁移完成: {input_path} -> {output_path}")
    print(f"节点键: {node_key}")
    print("新语义: axis.logical_indices[logical_axis] = physical_axis")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pylint: disable=broad-except
        print(f"迁移失败: {exc}", file=sys.stderr)
        raise
