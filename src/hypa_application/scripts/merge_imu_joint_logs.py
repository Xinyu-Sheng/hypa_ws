#!/usr/bin/env python3
"""按时间戳合并 IMU 和 joint_states CSV，并优先使用高频序列作为输出时间轴。

参数使用说明:

    必需参数:
        --imu PATH
                IMU CSV 文件路径。CSV 必须包含 `stamp_ns` 列（纳秒）。
        --joint PATH
                joint_states CSV 文件路径。CSV 必须包含 `stamp_ns` 列（纳秒）。
        --output PATH
                输出 CSV 文件路径。

    可选参数:
        --mode {overlap_timestamps,full_timestamps}
                输出模式（默认: full_timestamps）。
                - overlap_timestamps: 只输出两路时间轴的重叠部分（裁剪）。
                - full_timestamps: 保留选定的目标时间轴，但当另一路在该时间点无数据时写入 NaN（不做外推）。
        --max-gap-ratio N
                允许插值的最大间隔倍数（默认: 3.0）。若两点时间间隔超过 N*目标周期则该字段写 NaN。

    运行示例:
        python src/hypa_application/scripts/merge_imu_joint_logs.py \
            --imu /path/imu.csv \
            --joint /path/joint.csv \
            --output /tmp/merged.csv \
            --mode full_timestamps

输出说明:
    - 输出 CSV 包含原始字段，IMU 字段前缀为 `imu_`，joint 字段前缀为 `joint_`，并额外包含 `quat_x, quat_y, quat_z, quat_w`（若存在四元数）以及 `roll_deg, pitch_deg, yaw_deg`。
    - 对于姿态，若输入包含四元数，脚本会先对四元数执行 SLERP 插值，再转换为欧拉角输出，以避免欧拉角直接插值导致的跳变。
    - 脚本依赖: `numpy`。
"""

from __future__ import annotations

import argparse
import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class SampleSeries:
    stamps_ns: np.ndarray
    rows: List[dict]
    numeric_fields: List[str]


class OutputMode:
    OVERLAP_TIMESTAMPS = "overlap_timestamps"
    FULL_TIMESTAMPS = "full_timestamps"


def _load_csv(_path: Path) -> Tuple[List[dict], List[str]]:
    with _path.open("r", encoding="utf-8-sig", newline="") as file_handle:
        reader = csv.DictReader(file_handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV 文件没有表头: {_path}")
        rows = [row for row in reader]
        return rows, list(reader.fieldnames)


def _to_float(_value: str) -> float:
    text = str(_value).strip()
    if text == "":
        return float("nan")
    return float(text)


def _load_series(_path: Path) -> SampleSeries:
    rows, fieldnames = _load_csv(_path)
    if "stamp_ns" not in fieldnames:
        raise ValueError(f"CSV 缺少 stamp_ns 列: {_path}")

    stamps: List[int] = []
    cleaned_rows: List[dict] = []
    numeric_fields: List[str] = [name for name in fieldnames if name != "stamp_ns"]

    for row in rows:
        stamp_text = str(row.get("stamp_ns", "")).strip()
        if not stamp_text:
            continue
        try:
            stamp_ns = int(float(stamp_text))
        except ValueError as exc:
            raise ValueError(f"stamp_ns 解析失败: {stamp_text}") from exc
        stamps.append(stamp_ns)
        cleaned_rows.append(row)

    if not stamps:
        raise ValueError(f"CSV 没有有效数据行: {_path}")

    order = np.argsort(np.asarray(stamps, dtype=np.int64))
    sorted_stamps = np.asarray([stamps[index] for index in order], dtype=np.int64)
    sorted_rows = [cleaned_rows[index] for index in order]

    dedup_stamps: List[int] = []
    dedup_rows: List[dict] = []
    for stamp_ns, row in zip(sorted_stamps.tolist(), sorted_rows):
        if dedup_stamps and stamp_ns == dedup_stamps[-1]:
            dedup_rows[-1] = row
        else:
            dedup_stamps.append(stamp_ns)
            dedup_rows.append(row)

    return SampleSeries(
        stamps_ns=np.asarray(dedup_stamps, dtype=np.int64),
        rows=dedup_rows,
        numeric_fields=numeric_fields,
    )


def _estimate_period_ns(_stamps_ns: np.ndarray) -> float:
    if _stamps_ns.size < 2:
        return float("inf")
    diffs = np.diff(_stamps_ns.astype(np.float64))
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        return float("inf")
    return float(np.median(diffs))


def _clamp_index(_stamps_ns: np.ndarray, _stamp_ns: int) -> int:
    index = bisect_left(_stamps_ns.tolist(), _stamp_ns)
    if index <= 0:
        return 0
    if index >= _stamps_ns.size:
        return _stamps_ns.size - 1
    return index


def _linear_interp(_x0: float, _x1: float, _y0: float, _y1: float, _x: float) -> float:
    if math.isclose(_x0, _x1):
        return _y0
    ratio = (_x - _x0) / (_x1 - _x0)
    return _y0 + ratio * (_y1 - _y0)


def _normalize_quaternion(_q: Sequence[float]) -> np.ndarray:
    quat = np.asarray(_q, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def _slerp(_q0: Sequence[float], _q1: Sequence[float], _t: float) -> np.ndarray:
    q0 = _normalize_quaternion(_q0)
    q1 = _normalize_quaternion(_q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = q0 + _t * (q1 - q0)
        return _normalize_quaternion(result)

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    if math.isclose(sin_theta_0, 0.0):
        return q0

    theta = theta_0 * _t
    sin_theta = math.sin(theta)
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * q0 + s1 * q1


def _quaternion_to_euler_deg(
    _qx: float, _qy: float, _qz: float, _qw: float
) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (_qw * _qx + _qy * _qz)
    cosr_cosp = 1.0 - 2.0 * (_qx * _qx + _qy * _qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (_qw * _qy - _qz * _qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (_qw * _qz + _qx * _qy)
    cosy_cosp = 1.0 - 2.0 * (_qy * _qy + _qz * _qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _find_field_name(
    _fieldnames: Sequence[str], _candidates: Sequence[str]
) -> Optional[str]:
    lowered = {name.lower(): name for name in _fieldnames}
    for candidate in _candidates:
        if candidate.lower() in lowered:
            return lowered[candidate.lower()]
    return None


def _extract_numeric_row(_row: dict, _fieldnames: Sequence[str]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for field_name in _fieldnames:
        if field_name == "stamp_ns":
            continue
        try:
            result[field_name] = _to_float(_row.get(field_name, ""))
        except ValueError:
            result[field_name] = float("nan")
    return result


def _extract_quaternion_fields(
    _row: dict,
    _fieldnames: Sequence[str],
) -> Optional[Tuple[float, float, float, float]]:
    quaternion_candidates = [
        ("orientation.x", "orientation.y", "orientation.z", "orientation.w"),
        ("qx", "qy", "qz", "qw"),
        ("quat_x", "quat_y", "quat_z", "quat_w"),
        ("imu_qx", "imu_qy", "imu_qz", "imu_qw"),
    ]
    for field_names in quaternion_candidates:
        if all(field_name in _fieldnames for field_name in field_names):
            try:
                return tuple(_to_float(_row.get(field_name, "")) for field_name in field_names)  # type: ignore[return-value]
            except ValueError:
                return None
    return None


def _interpolate_quaternion(
    _series: SampleSeries,
    _target_stamp_ns: int,
    _max_gap_ns: int,
    _mode: str,
) -> Optional[Tuple[float, float, float, float]]:
    stamps = _series.stamps_ns
    rows = _series.rows

    lower_index = bisect_left(stamps.tolist(), _target_stamp_ns)
    if lower_index <= 0:
        if _mode == OutputMode.OVERLAP_TIMESTAMPS:
            return None
        return _extract_quaternion_fields(rows[0], _series.numeric_fields)
    if lower_index >= stamps.size:
        if _mode == OutputMode.OVERLAP_TIMESTAMPS:
            return None
        return _extract_quaternion_fields(rows[-1], _series.numeric_fields)

    upper_index = lower_index
    lower_index = upper_index - 1

    lower_stamp = int(stamps[lower_index])
    upper_stamp = int(stamps[upper_index])
    if upper_stamp - lower_stamp > _max_gap_ns:
        return None

    lower_quat = _extract_quaternion_fields(rows[lower_index], _series.numeric_fields)
    upper_quat = _extract_quaternion_fields(rows[upper_index], _series.numeric_fields)
    if lower_quat is None or upper_quat is None:
        return None

    if upper_stamp == lower_stamp:
        return lower_quat

    ratio = (_target_stamp_ns - lower_stamp) / (upper_stamp - lower_stamp)
    quat = _slerp(lower_quat, upper_quat, ratio)
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def _interpolate_row(
    _series: SampleSeries,
    _target_stamp_ns: int,
    _max_gap_ns: int,
    _mode: str,
) -> Dict[str, float]:
    stamps = _series.stamps_ns
    rows = _series.rows

    if _target_stamp_ns <= int(stamps[0]):
        if _mode == OutputMode.OVERLAP_TIMESTAMPS:
            return {field_name: float("nan") for field_name in _series.numeric_fields}
        return _extract_numeric_row(rows[0], _series.numeric_fields)
    if _target_stamp_ns >= int(stamps[-1]):
        if _mode == OutputMode.OVERLAP_TIMESTAMPS:
            return {field_name: float("nan") for field_name in _series.numeric_fields}
        return _extract_numeric_row(rows[-1], _series.numeric_fields)

    upper_index = _clamp_index(stamps, _target_stamp_ns)
    if int(stamps[upper_index]) <= _target_stamp_ns:
        lower_index = upper_index
        upper_index = min(upper_index + 1, stamps.size - 1)
    else:
        lower_index = max(upper_index - 1, 0)

    lower_stamp = int(stamps[lower_index])
    upper_stamp = int(stamps[upper_index])
    if upper_stamp - lower_stamp > _max_gap_ns:
        return {field_name: float("nan") for field_name in _series.numeric_fields}

    lower_values = _extract_numeric_row(rows[lower_index], _series.numeric_fields)
    upper_values = _extract_numeric_row(rows[upper_index], _series.numeric_fields)
    ratio = (
        0.0
        if upper_stamp == lower_stamp
        else (_target_stamp_ns - lower_stamp) / (upper_stamp - lower_stamp)
    )

    interpolated: Dict[str, float] = {}
    for field_name in _series.numeric_fields:
        lower_value = lower_values.get(field_name, float("nan"))
        upper_value = upper_values.get(field_name, float("nan"))
        if math.isnan(lower_value):
            interpolated[field_name] = upper_value
        elif math.isnan(upper_value):
            interpolated[field_name] = lower_value
        else:
            interpolated[field_name] = _linear_interp(
                lower_stamp, upper_stamp, lower_value, upper_value, _target_stamp_ns
            )
    return interpolated


def _merge_series(
    _imu_series: SampleSeries,
    _joint_series: SampleSeries,
    _output_path: Path,
    _max_gap_ratio: float,
    _mode: str,
) -> None:
    imu_period_ns = _estimate_period_ns(_imu_series.stamps_ns)
    joint_period_ns = _estimate_period_ns(_joint_series.stamps_ns)

    if imu_period_ns <= joint_period_ns:
        target_series = _imu_series
        other_series = _joint_series
        target_label = "imu"
    else:
        target_series = _joint_series
        other_series = _imu_series
        target_label = "joint_states"

    target_period_ns = _estimate_period_ns(target_series.stamps_ns)
    if not math.isfinite(target_period_ns) or target_period_ns <= 0.0:
        target_period_ns = min(imu_period_ns, joint_period_ns)

    max_gap_ns = int(max(target_period_ns, 1.0) * _max_gap_ratio)
    if _mode == OutputMode.OVERLAP_TIMESTAMPS:
        start_stamp = max(
            int(_imu_series.stamps_ns[0]), int(_joint_series.stamps_ns[0])
        )
        end_stamp = min(
            int(_imu_series.stamps_ns[-1]), int(_joint_series.stamps_ns[-1])
        )
        output_stamps = [
            stamp
            for stamp in target_series.stamps_ns.tolist()
            if start_stamp <= stamp <= end_stamp
        ]
    else:
        output_stamps = target_series.stamps_ns.tolist()

    other_has_quat = (
        _extract_quaternion_fields(other_series.rows[0], other_series.numeric_fields)
        is not None
    )

    _output_path.parent.mkdir(parents=True, exist_ok=True)
    with _output_path.open("w", encoding="utf-8", newline="") as file_handle:
        writer = csv.writer(file_handle)
        header: List[str] = ["stamp_ns", "source"]
        header.extend(
            [f"imu_{field_name}" for field_name in _imu_series.numeric_fields]
        )
        header.extend(
            [f"joint_{field_name}" for field_name in _joint_series.numeric_fields]
        )
        header.extend(["quat_x", "quat_y", "quat_z", "quat_w"])
        header.extend(["roll_deg", "pitch_deg", "yaw_deg"])
        writer.writerow(header)

        for stamp_ns in output_stamps:
            imu_values = _interpolate_row(_imu_series, int(stamp_ns), max_gap_ns, _mode)
            joint_values = _interpolate_row(
                _joint_series, int(stamp_ns), max_gap_ns, _mode
            )

            quat_x = float("nan")
            quat_y = float("nan")
            quat_z = float("nan")
            quat_w = float("nan")
            roll_deg = float("nan")
            pitch_deg = float("nan")
            yaw_deg = float("nan")

            if other_series is _imu_series and other_has_quat:
                quat = _interpolate_quaternion(
                    other_series, int(stamp_ns), max_gap_ns, _mode
                )
                if quat is not None:
                    quat_x, quat_y, quat_z, quat_w = quat
                    roll_deg, pitch_deg, yaw_deg = _quaternion_to_euler_deg(
                        quat_x,
                        quat_y,
                        quat_z,
                        quat_w,
                    )
            row: List[object] = [int(stamp_ns), target_label]
            row.extend(
                imu_values.get(field_name, float("nan"))
                for field_name in _imu_series.numeric_fields
            )
            row.extend(
                joint_values.get(field_name, float("nan"))
                for field_name in _joint_series.numeric_fields
            )
            row.extend([quat_x, quat_y, quat_z, quat_w])
            row.extend([roll_deg, pitch_deg, yaw_deg])
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="合并 IMU 与 joint_states CSV，自动选高频率并插值对齐。"
    )
    parser.add_argument("--imu", required=True, type=Path, help="IMU CSV 文件路径")
    parser.add_argument(
        "--joint", required=True, type=Path, help="joint_states CSV 文件路径"
    )
    parser.add_argument("--output", required=True, type=Path, help="输出 CSV 文件路径")
    parser.add_argument(
        "--mode",
        choices=[OutputMode.OVERLAP_TIMESTAMPS, OutputMode.FULL_TIMESTAMPS],
        default=OutputMode.FULL_TIMESTAMPS,
        help="输出模式：overlap_timestamps 只保留重叠时间轴；full_timestamps 保留目标时间轴但越界填 NaN。默认 full_timestamps",
    )
    parser.add_argument(
        "--max-gap-ratio",
        type=float,
        default=3.0,
        help="允许插值的最大间隔倍数，超过则写入 NaN。默认 3.0",
    )
    args = parser.parse_args()

    imu_series = _load_series(args.imu)
    joint_series = _load_series(args.joint)
    _merge_series(
        imu_series,
        joint_series,
        args.output,
        args.max_gap_ratio,
        args.mode,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
