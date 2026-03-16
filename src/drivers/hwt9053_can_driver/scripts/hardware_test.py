#!/usr/bin/env python3
"""
HWT9053 CAN 驱动硬件测试脚本

用途: 验证HWT9053传感器硬件集成和驱动功能

使用:
  python3 hardware_test.py                    # 运行所有测试
  python3 hardware_test.py --test can_check   # 仅检查CAN接口
  python3 hardware_test.py --robot robot1    # 指定机器人
  python3 hardware_test.py --duration 30     # 测试持续时间（秒）

版本: v0.0.2
日期: 2026-03-16
"""

import sys
import argparse
import subprocess
import time
import math
import statistics
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header


class TestStatus(Enum):
    """测试状态枚举"""

    UNKNOWN = "⚪"
    PASS = "✅"
    FAIL = "❌"
    SKIP = "⊘"
    WARNING = "⚠️"


@dataclass
class TestResult:
    """测试结果数据类"""

    name: str
    status: TestStatus
    message: str
    duration: float = 0.0


class HWT9053HardwareTestor(Node):
    """HWT9053硬件测试节点"""

    def __init__(self, robot_name: str = "robot"):
        super().__init__("hwt9053_hardware_test")
        self.robot_name = robot_name
        self.results: List[TestResult] = []

        # 数据收集
        self.imu_messages: List[Imu] = []
        self.mag_messages: List[MagneticField] = []
        self.test_duration = 10.0  # 秒
        self.start_time = None

        # 订阅话题
        self.imu_topic = f"/{robot_name}/imu/data"
        self.mag_topic = f"/{robot_name}/imu/data_mag"

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, self.mag_topic, self.mag_callback, 10
        )

        self.get_logger().info(f"硬件测试节点已创建，监听 {self.imu_topic}")

    def imu_callback(self, msg: Imu):
        """IMU消息回调"""
        if self.start_time is None:
            self.start_time = time.time()
        self.imu_messages.append(msg)

    def mag_callback(self, msg: MagneticField):
        """磁场消息回调"""
        self.mag_messages.append(msg)


def run_shell_command(cmd: str, check: bool = True) -> Tuple[int, str, str]:
    """运行shell命令"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=10
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "命令超时"
    except Exception as e:
        return -1, "", str(e)


def check_can_interface(interface: str) -> TestResult:
    """检查CAN接口"""
    test_name = f"CAN接口检查 ({interface})"
    start_time = time.time()

    # 1. 检查接口是否存在
    ret, stdout, stderr = run_shell_command(f"ip link show {interface}")
    if ret != 0:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            f"CAN接口不存在: {interface}",
            time.time() - start_time,
        )

    # 2. 检查接口是否UP
    if "UP" not in stdout or "RUNNING" not in stdout:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            f"CAN接口未启动: {interface}。运行: sudo ip link set {interface} up",
            time.time() - start_time,
        )

    # 3. 检查波特率
    if "500000" in stdout:
        rate_info = "500kbps"
    elif "250000" in stdout:
        rate_info = "250kbps"
    else:
        rate_info = "未知"

    return TestResult(
        test_name,
        TestStatus.PASS,
        f"CAN接口正常，波特率: {rate_info}",
        time.time() - start_time,
    )


def check_can_messages(interface: str, timeout: float = 5.0) -> TestResult:
    """检查CAN消息流"""
    test_name = f"CAN消息流检查 ({interface})"
    start_time = time.time()

    # 启动candump进程
    try:
        process = subprocess.Popen(
            f"candump {interface}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(1)  # 等待进程启动

        # 收集消息
        messages = []
        end_time = time.time() + timeout

        while time.time() < end_time:
            if process.poll() is not None:
                break
            line = process.stdout.readline() if process.stdout else ""
            if line and ">" in line:  # CAN消息格式: "can0 050"
                messages.append(line.strip())
                if len(messages) >= 5:
                    break
            time.sleep(0.01)

        process.terminate()

        if len(messages) >= 5:
            # 统计消息ID
            can_ids = set()
            for msg in messages:
                if len(msg.split()) > 1:
                    can_ids.add(msg.split()[1])

            return TestResult(
                test_name,
                TestStatus.PASS,
                f"检测到 {len(messages)} 条CAN消息，CAN IDs: {', '.join(sorted(can_ids))}",
                time.time() - start_time,
            )
        else:
            return TestResult(
                test_name,
                TestStatus.FAIL,
                f"未检测到CAN消息（<5条在{timeout}s内）",
                time.time() - start_time,
            )

    except Exception as e:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            f"CAN消息检查失败: {str(e)}",
            time.time() - start_time,
        )


def check_ros2_nodes(robot_name: str) -> TestResult:
    """检查ROS 2节点"""
    test_name = "ROS 2节点检查"
    start_time = time.time()

    ret, stdout, stderr = run_shell_command("ros2 node list")
    if ret != 0:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            "无法列出ROS 2节点（检查ROS 2环境）",
            time.time() - start_time,
        )

    expected_node = f"/{robot_name}/hwt9053_can_driver"
    if expected_node in stdout:
        return TestResult(
            test_name,
            TestStatus.PASS,
            f"发现节点: {expected_node}",
            time.time() - start_time,
        )
    else:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            f"未发现节点: {expected_node}（请先启动驱动）",
            time.time() - start_time,
        )


def check_ros2_topics(robot_name: str) -> TestResult:
    """检查ROS 2话题"""
    test_name = "ROS 2话题检查"
    start_time = time.time()

    ret, stdout, stderr = run_shell_command("ros2 topic list")
    if ret != 0:
        return TestResult(
            test_name, TestStatus.FAIL, "无法列出ROS 2话题", time.time() - start_time
        )

    imu_topic = f"/{robot_name}/imu/data"
    mag_topic = f"/{robot_name}/imu/data_mag"

    found_topics = []
    if imu_topic in stdout:
        found_topics.append(f"✅ {imu_topic}")
    else:
        found_topics.append(f"❌ {imu_topic}")

    if mag_topic in stdout:
        found_topics.append(f"✅ {mag_topic}")
    else:
        found_topics.append(f"⚠️ {mag_topic} (可选)")

    status = TestStatus.PASS if imu_topic in stdout else TestStatus.FAIL
    message = "\n  ".join(found_topics)

    return TestResult(test_name, status, message, time.time() - start_time)


def test_imu_data(tester: HWT9053HardwareTestor) -> TestResult:
    """测试IMU数据质量"""
    test_name = "IMU数据质量测试"
    start_time = time.time()

    # 清空之前的数据
    tester.imu_messages.clear()
    tester.mag_messages.clear()
    tester.start_time = None

    # 收集数据
    end_time = time.time() + tester.test_duration
    while time.time() < end_time:
        rclpy.spin_once(tester, timeout_sec=0.1)

    if len(tester.imu_messages) < 10:
        return TestResult(
            test_name,
            TestStatus.FAIL,
            f"收集IMU消息不足：{len(tester.imu_messages)} 条（期望 ~{int(tester.test_duration * 200)}）",
            time.time() - start_time,
        )

    # 检查数据范围
    accel_x_values = [m.linear_acceleration.x for m in tester.imu_messages]
    accel_y_values = [m.linear_acceleration.y for m in tester.imu_messages]
    accel_z_values = [m.linear_acceleration.z for m in tester.imu_messages]

    # 加速度应在±19.62 m/s²范围内（±2g）
    valid_accel = all(
        -25 < x < 25 and -25 < y < 25 and -25 < z < 25
        for x, y, z in zip(accel_x_values, accel_y_values, accel_z_values)
    )

    if not valid_accel:
        return TestResult(
            test_name, TestStatus.FAIL, f"加速度数据超出范围", time.time() - start_time
        )

    # 四元数归一化检查
    quat_norms = []
    for m in tester.imu_messages:
        norm = math.sqrt(
            m.orientation.w**2
            + m.orientation.x**2
            + m.orientation.y**2
            + m.orientation.z**2
        )
        quat_norms.append(norm)

    avg_norm = statistics.mean(quat_norms)
    if abs(avg_norm - 1.0) > 0.01:  # 允许1%的误差
        return TestResult(
            test_name,
            TestStatus.WARNING,
            f"四元数未完全归一化：平均 {avg_norm:.4f} （期望 1.0）",
            time.time() - start_time,
        )

    # 频率检查
    msg_rate = len(tester.imu_messages) / tester.test_duration
    expected_rate = 200
    actual_rate_error = abs(msg_rate - expected_rate) / expected_rate * 100

    if actual_rate_error > 15:  # 允许15%的误差
        status = TestStatus.WARNING
    else:
        status = TestStatus.PASS

    message = (
        f"✅ 收集 {len(tester.imu_messages)} 条消息\n"
        f"  频率: {msg_rate:.1f} Hz （相对误差: {actual_rate_error:.1f}%）\n"
        f"  四元数范数: {avg_norm:.4f}\n"
        f"  加速度范围: "
        f"X=[{min(accel_x_values):.3f}, {max(accel_x_values):.3f}], "
        f"Y=[{min(accel_y_values):.3f}, {max(accel_y_values):.3f}], "
        f"Z=[{min(accel_z_values):.3f}, {max(accel_z_values):.3f}]"
    )

    return TestResult(test_name, status, message, time.time() - start_time)


def test_magnetic_field(tester: HWT9053HardwareTestor) -> TestResult:
    """测试磁场数据"""
    test_name = "磁场数据测试"
    start_time = time.time()

    if len(tester.mag_messages) == 0:
        return TestResult(
            test_name,
            TestStatus.WARNING,
            "未接收到磁场消息（可能未发布或未启用）",
            time.time() - start_time,
        )

    # 检查磁场范围
    mag_x_values = [m.magnetic_field.x for m in tester.mag_messages]
    mag_y_values = [m.magnetic_field.y for m in tester.mag_messages]
    mag_z_values = [m.magnetic_field.z for m in tester.mag_messages]

    # 应在±5200µT范围内（±400µT量程的±2倍）
    valid_mag = all(
        -6000 < x < 6000 and -6000 < y < 6000 and -6000 < z < 6000
        for x, y, z in zip(mag_x_values, mag_y_values, mag_z_values)
    )

    if not valid_mag:
        return TestResult(
            test_name,
            TestStatus.WARNING,
            "磁场数据超出通常范围",
            time.time() - start_time,
        )

    message = (
        f"✅ 收集 {len(tester.mag_messages)} 条消息\n"
        f"  X: [{min(mag_x_values):.2f}, {max(mag_x_values):.2f}] µT\n"
        f"  Y: [{min(mag_y_values):.2f}, {max(mag_y_values):.2f}] µT\n"
        f"  Z: [{min(mag_z_values):.2f}, {max(mag_z_values):.2f}] µT"
    )

    return TestResult(test_name, TestStatus.PASS, message, time.time() - start_time)


def print_results(results: List[TestResult]):
    """打印测试结果"""
    print("\n" + "=" * 70)
    print("HWT9053 硬件集成测试结果")
    print("=" * 70)

    # 按状态分组
    passed = sum(1 for r in results if r.status == TestStatus.PASS)
    failed = sum(1 for r in results if r.status == TestStatus.FAIL)
    warnings = sum(1 for r in results if r.status == TestStatus.WARNING)

    for result in results:
        print(f"\n{result.status.value} {result.name} ({result.duration:.2f}s)")
        for line in result.message.split("\n"):
            print(f"   {line}")

    print("\n" + "=" * 70)
    print(f"测试汇总: {passed} 通过 | {warnings} 警告 | {failed} 失败")
    print("=" * 70)

    return failed == 0


def main():
    """主测试函数"""
    parser = argparse.ArgumentParser(description="HWT9053硬件集成测试")
    parser.add_argument("--can-interface", default="can0", help="CAN接口（默认：can0）")
    parser.add_argument("--robot", default="robot", help="机器人名称（默认：robot）")
    parser.add_argument(
        "--duration", type=float, default=10.0, help="测试持续时间（秒）"
    )
    parser.add_argument("--test", help="仅运行特定测试（如：can_check, ros2_check）")

    args = parser.parse_args()

    results = []

    # 测试1: CAN接口
    if args.test is None or "can" in args.test:
        print(f"检查CAN接口 {args.can_interface} ...")
        results.append(check_can_interface(args.can_interface))

        if results[-1].status != TestStatus.FAIL:
            print("检查CAN消息流 ...")
            results.append(check_can_messages(args.can_interface))

    # 测试2: ROS 2
    if args.test is None or "ros2" in args.test:
        print("检查ROS 2节点 ...")
        results.append(check_ros2_nodes(args.robot))

        print("检查ROS 2话题 ...")
        results.append(check_ros2_topics(args.robot))

    # 测试3: 数据质量（需要ROS 2初始化）
    if args.test is None or "data" in args.test:
        try:
            rclpy.init()
            tester = HWT9053HardwareTestor(args.robot)
            tester.test_duration = args.duration

            print(f"收集IMU数据（{args.duration}秒） ...")
            results.append(test_imu_data(tester))

            if args.robot != "robot":  # 仅在指定时检查磁场
                print("检查磁场数据 ...")
                results.append(test_magnetic_field(tester))

            rclpy.shutdown()
        except Exception as e:
            results.append(
                TestResult("数据质量测试", TestStatus.FAIL, f"测试异常: {str(e)}", 0.0)
            )

    # 打印结果
    success = print_results(results)

    # 返回exit code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
