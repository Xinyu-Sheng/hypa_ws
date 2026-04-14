import math
import serial
import struct
import numpy as np
import rclpy
import subprocess
import time
import threading
import os
from datetime import datetime
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from imu_msg.msg import ImuData


class protocolType(Enum):
    TTL_STD = 1  # TTL-标准精度
    TTL_HIGH = 2  # TTL-高精度
    CAN_STD = 3  # CAN-标准精度
    CAN_HIGH = 4  # CAN-高精度
    RS485_STD = 5  # RS485-标准精度
    RS485_HIGH = 6  # RS485-高精度


class imuDriverNode(Node):
    def __init__(self):
        super().__init__("imuDriverNode")

        # 参数
        self.declare_parameter("namespace", "")
        # self.declare_parameter("use_sim_time", False)
        self.declare_parameter("port", "/dev/imu_usb")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("protocol", "RS485_HIGH")
        self.declare_parameter("modbusID", 0x50)
        self.declare_parameter("mag_topic", "imu/mag")
        self.declare_parameter("publish_mag", True)
        # CSV logging 参数（最小侵入）
        # 使用合并参数 record_imu_csv_path（形如 "/tmp/imu_data"），不再向后兼容旧的 dir/file 参数
        self.declare_parameter("record_imu_csv_enabled", False)
        self.declare_parameter("record_imu_csv_path", "/tmp/imu_data")
        self.declare_parameter("record_imu_csv_buffer_size", 100)
        self.declare_parameter("record_imu_csv_flush_interval_s", 1.0)
        # watchdog 与 lifecycle 控制参数（最小侵入）
        self.declare_parameter("imu_watchdog_timeout", 1.0)  # 秒, 0 或负值禁用
        self.declare_parameter("deactivate_zmotion_on_fault", True)
        self.declare_parameter("zmotion_driver_node_name", "zmotion_driver")

        self.namespace_param = self.get_parameter("namespace").value
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.modbusID = self.get_parameter("modbusID").value
        protocolStr = self.get_parameter("protocol").value
        self.mag_topic = self.get_parameter("mag_topic").value
        self.publish_mag = self.get_parameter("publish_mag").value

        self.protocol = protocolType[protocolStr]

        # CSV 记录配置（文件名使用系统本机时间作为后缀）
        self._csv_enabled = bool(self.get_parameter("record_imu_csv_enabled").value)
        # 仅使用 record_imu_csv_path 参数（示例："/tmp/imu_data"）；若为空则使用默认 "/tmp/imu_data"
        csv_path_param = str(self.get_parameter("record_imu_csv_path").value).strip()
        if csv_path_param:
            csv_dir = os.path.dirname(csv_path_param)
            csv_base = os.path.basename(csv_path_param)
            if csv_dir == "":
                csv_dir = "/tmp"
        else:
            csv_dir = "/tmp"
            csv_base = "imu_data"
        name, ext = os.path.splitext(csv_base)
        self._csv_dir = csv_dir
        self._csv_basename = name if name else csv_base

        self._csv_buffer_size = int(
            self.get_parameter("record_imu_csv_buffer_size").value
        )
        self._csv_flush_interval_s = float(
            self.get_parameter("record_imu_csv_flush_interval_s").value
        )

        # CSV 写入运行时结构
        self._csv_buffer = []
        self._csv_lock = threading.Lock()
        self._csv_event = threading.Event()
        self._csv_stop_event = threading.Event()
        self._csv_thread = None
        self._csv_file = None

        if self._csv_enabled:
            try:
                os.makedirs(self._csv_dir, exist_ok=True)
            except Exception as e:
                self.get_logger().warning(
                    f"Could not create CSV dir {self._csv_dir}: {e}"
                )
                self._csv_enabled = False

        if self._csv_enabled:
            try:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                file_name = f"{self._csv_basename}_{ts}.csv"
                file_path = os.path.join(self._csv_dir, file_name)
                self._csv_file = open(file_path, "a", encoding="utf-8")
                header = "stamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,roll_deg,pitch_deg,yaw_deg,mag_x,mag_y,mag_z\n"
                self._csv_file.write(header)
                self._csv_file.flush()
                self._csv_thread = threading.Thread(
                    target=self._csv_writer_thread, daemon=True
                )
                self._csv_thread.start()
                self.get_logger().info(f"IMU CSV logging enabled -> {file_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to open CSV file {file_path}: {e}")
                self._csv_enabled = False

        # watchdog 状态与时间戳（纳秒）
        self.imu_watchdog_timeout = float(
            self.get_parameter("imu_watchdog_timeout").value
        )
        self.deactivate_zmotion_on_fault = bool(
            self.get_parameter("deactivate_zmotion_on_fault").value
        )
        self.zmotion_driver_node_name = self.get_parameter(
            "zmotion_driver_node_name"
        ).value

        self.last_success_ts = self.get_clock().now().nanoseconds
        self._fault_reported = False

        # 串口
        try:
            self.serialPort = serial.Serial(
                self.port, self.baudrate, timeout=0.001, write_timeout=0
            )
            self.get_logger().info(f"Serial opened: {self.port} @ {self.baudrate}")
        except Exception as e:
            self.serialPort = None
            self.get_logger().warning(
                f"Could not open serial port {self.port}: {e}. Running without serial input."
            )

        self.get_logger().info(f"Configured with namespace={self.namespace_param}")

        self.modbusAddrList = [0x34, 0x37, 0x3A, 0x3D]
        self.modbusIndex = 0
        self.waitingResponse = False
        self.lastSendTime = 0

        # 接收缓存
        self.rxBuffer = bytearray()

        # 数据缓存
        self.accData = np.zeros(3)
        self.gyroData = np.zeros(3)
        self.angleData = np.zeros(3)
        self.magData = np.zeros(3)
        self.currentRequestAddr = None  # 标记 485 当前询问地址

        # 处理后的数据
        self.acc = np.zeros(3)
        self.gyro = np.zeros(3)
        self.angle = np.zeros(3)
        self.mag = np.zeros(3)

        # 发布器
        self.imuPublisher = self.create_publisher(Imu, "imu/data", 10)
        self.imuRpyPublisher = self.create_publisher(ImuData, "imu/ImuDataWithRPY", 10)
        self.magPublisher = None
        if self.publish_mag:
            self.magPublisher = self.create_publisher(MagneticField, self.mag_topic, 10)

        # 定时器
        self.create_timer(0.001, self.timerCallback)
        self.create_timer(0.1, self.printMsg)  # 终端打印
        # watchdog 定时器（如果启用）
        if self.imu_watchdog_timeout > 0:
            # 检查频率不需要太高，500ms 足够
            self.create_timer(0.5, self.watchdogCallback)

    # ==========================================================
    # 主循环
    # ==========================================================
    def timerCallback(self):
        # 仅当串口可用时执行 RS485 发送/超时逻辑
        if self.serialPort is not None and self.protocol in (
            protocolType.RS485_STD,
            protocolType.RS485_HIGH,
        ):
            now = self.get_clock().now().nanoseconds

            # 若没有等待响应，则发送新命令
            if not self.waitingResponse:
                addr = self.modbusAddrList[self.modbusIndex]
                self.currentRequestAddr = addr
                if self.protocol == protocolType.RS485_HIGH and addr == 0x3D:
                    cmd = self.buildModbusReadCmd(addr, 6)
                else:
                    cmd = self.buildModbusReadCmd(addr, 3)

                try:
                    self.serialPort.write(cmd)
                except Exception:
                    # 写失败则忽略，等待下次重试
                    pass

                self.waitingResponse = True
                self.lastSendTime = now

            # 超时保护（5ms）
            elif now - self.lastSendTime > 50000000:
                self.waitingResponse = False
                self.currentRequestAddr = None
                self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)

        self.readSerial()
        self.parseBuffer()

    # ==========================================================
    # 串口读取
    # ==========================================================
    def readSerial(self):
        # 如果串口不可用则直接返回
        if self.serialPort is None:
            return

        # 访问 in_waiting 可能在底层发生 IO 错误（例如拔线），因此单独捕获
        try:
            byteCount = self.serialPort.in_waiting
        except Exception as e:
            # 串口底层错误（如设备被拔出），记录并触发 zmotion_driver 停用（仅一次）
            self.get_logger().error(f"Serial port read availability error: {e}")
            if (
                not getattr(self, "_fault_reported", False)
                and self.deactivate_zmotion_on_fault
            ):
                ns = (
                    str(self.namespace_param).strip("/") if self.namespace_param else ""
                )
                if ns:
                    target = f"/{ns}/{self.zmotion_driver_node_name}"
                else:
                    target = self.zmotion_driver_node_name
                try:
                    self._fault_reported = True
                    self._call_deactivate_and_log(target)
                except Exception as ex:
                    self.get_logger().error(f"Exception calling lifecycle CLI: {ex}")

            # 请求 ROS 事件循环退出，保持与 watchdog 一致的优雅终止行为
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        if byteCount > 0:
            try:
                data = self.serialPort.read(byteCount)
                self.rxBuffer.extend(data)
            except Exception as e:
                # 读取失败则记录并在必要时触发停用
                self.get_logger().warning(f"Serial read failed: {e}")
                if (
                    not getattr(self, "_fault_reported", False)
                    and self.deactivate_zmotion_on_fault
                ):
                    ns = (
                        str(self.namespace_param).strip("/")
                        if self.namespace_param
                        else ""
                    )
                    if ns:
                        target = f"/{ns}/{self.zmotion_driver_node_name}"
                    else:
                        target = self.zmotion_driver_node_name
                    try:
                        self._fault_reported = True
                        self._call_deactivate_and_log(target)
                    except Exception as ex:
                        self.get_logger().error(
                            f"Exception calling lifecycle CLI: {ex}"
                        )

    # ==========================================================
    # buffer解析
    # ==========================================================
    def parseBuffer(self):
        while True:
            if self.protocol in (protocolType.TTL_STD, protocolType.TTL_HIGH):
                if len(self.rxBuffer) < 11:
                    return

                if self.rxBuffer[0] != 0x55:
                    self.rxBuffer.pop(0)
                    continue

                frame = self.rxBuffer[:11]
                del self.rxBuffer[:11]

                if not self.ttlChecksum(frame):
                    continue

                self.handleTTLFrame(frame)

            elif self.protocol in (protocolType.CAN_STD, protocolType.CAN_HIGH):
                if len(self.rxBuffer) < 8:
                    return

                if self.rxBuffer[0] != 0x55:
                    self.rxBuffer.pop(0)
                    continue

                frame = self.rxBuffer[:8]
                del self.rxBuffer[:8]

                self.handleCanFrame(frame)

            elif self.protocol in (protocolType.RS485_STD, protocolType.RS485_HIGH):
                if len(self.rxBuffer) < 5:
                    return

                # 地址不对，丢 1 字节重新对齐
                if self.rxBuffer[0] != self.modbusID:
                    self.rxBuffer.pop(0)
                    continue

                # 功能码错误，丢1字节
                if self.rxBuffer[1] != 0x03:
                    self.rxBuffer.pop(0)
                    continue

                # 读取字节数
                byteCount = self.rxBuffer[2]
                frameLen = 3 + byteCount + 2

                if len(self.rxBuffer) < frameLen:
                    return

                frame = self.rxBuffer[:frameLen]

                # CRC 校验
                recvCRC = (frame[-2] << 8) | frame[-1]
                calcCRC = self.modbusCRC(frame[:-2])
                calcCRC = ((calcCRC & 0xFF) << 8) | ((calcCRC >> 8) & 0xFF)

                if recvCRC != calcCRC:
                    self.rxBuffer.pop(0)  # CRC 错误，丢 1 字节重新对齐
                    continue

                # 通过校验，删除该帧
                del self.rxBuffer[:frameLen]

                self.handleModbusFrame(frame)

    # ==========================================================
    # TTL处理
    # ==========================================================
    def handleTTLFrame(self, frame):
        dataType = frame[1]

        if self.protocol == protocolType.TTL_STD:
            values = struct.unpack("<hhh", frame[2:8])

            if dataType == 0x51:
                self.accData = np.array(values)
            elif dataType == 0x52:
                self.gyroData = np.array(values)
            elif dataType == 0x53:
                self.angleData = np.array(values)
            elif dataType == 0x54:
                self.magData = np.array(values)
        else:
            # 普通 16bit 数据（acc/gyro/mag）
            if dataType in (0x51, 0x52, 0x54):
                values = struct.unpack("<hhh", frame[2:8])

                if dataType == 0x51:
                    self.accData = np.array(values)
                elif dataType == 0x52:
                    self.gyroData = np.array(values)
                elif dataType == 0x54:
                    self.magData = np.array(values)

            # 高精度 32bit 数据
            elif dataType == 0x53:
                low16 = struct.unpack("<H", frame[4:6])[0]  # 读取低 16bit
                high16 = struct.unpack("<h", frame[6:8])[0]  # 读取高 16bit
                angle32 = (high16 << 16) | low16

                # 写入对应轴
                axis = frame[2]
                if axis == 0x01:
                    self.angleData[0] = angle32
                elif axis == 0x02:
                    self.angleData[1] = angle32
                elif axis == 0x03:
                    self.angleData[2] = angle32

        self.publishImu()

    # ==========================================================
    # CAN处理
    # ==========================================================
    def handleCanFrame(self, frame):
        dataType = frame[1]

        if self.protocol == protocolType.CAN_STD:
            values = struct.unpack("<hhh", frame[2:8])

            if dataType == 0x51:
                self.accData = np.array(values)
            elif dataType == 0x52:
                self.gyroData = np.array(values)
            elif dataType == 0x53:
                self.angleData = np.array(values)
            elif dataType == 0x54:
                self.magData = np.array(values)
        else:
            # 普通 16bit 数据（acc/gyro/mag）
            if dataType in (0x51, 0x52, 0x54):
                values = struct.unpack("<hhh", frame[2:8])

                if dataType == 0x51:
                    self.accData = np.array(values)
                elif dataType == 0x52:
                    self.gyroData = np.array(values)
                elif dataType == 0x54:
                    self.magData = np.array(values)

            # 高精度 32bit 数据
            elif dataType == 0x53:
                low16 = struct.unpack("<H", frame[4:6])[0]  # 读取低 16bit
                high16 = struct.unpack("<h", frame[6:8])[0]  # 读取高 16bit
                angle32 = (high16 << 16) | low16

                # 写入对应轴
                axis = frame[2]
                if axis == 0x01:
                    self.angleData[0] = angle32
                elif axis == 0x02:
                    self.angleData[1] = angle32
                elif axis == 0x03:
                    self.angleData[2] = angle32

        self.publishImu()

    # ==========================================================
    # RS485处理
    # ==========================================================
    def handleModbusFrame(self, frame):
        if len(frame) < 5:
            return

        byteCount = frame[2]
        data = frame[3 : 3 + byteCount]

        addr = self.currentRequestAddr

        if self.protocol == protocolType.RS485_STD:
            if len(data) != 6:
                return

            values = struct.unpack(">hhh", data)

            if addr == 0x34:
                self.accData = np.array(values)
            elif addr == 0x37:
                self.gyroData = np.array(values)
            elif addr == 0x3A:
                self.magData = np.array(values)
            elif addr == 0x3D:
                self.angleData = np.array(values)
        else:
            if addr == 0x3D:  # 角度
                if len(data) != 12:
                    return

                def parse32(offset):
                    raw = (
                        (data[offset + 2] << 24)
                        | (data[offset + 3] << 16)
                        | (data[offset + 0] << 8)
                        | data[offset + 1]
                    )
                    if raw & 0x80000000:
                        raw -= 0x100000000
                    return raw

                # xLow  = struct.unpack(">H", data[0:2])[0]
                # xHigh = struct.unpack(">h", data[2:4])[0]

                # yLow  = struct.unpack(">H", data[4:6])[0]
                # yHigh = struct.unpack(">h", data[6:8])[0]

                # zLow  = struct.unpack(">H", data[8:10])[0]
                # zHigh = struct.unpack(">h", data[10:12])[0]

                # angleX = (xHigh << 16) | xLow
                # angleY = (yHigh << 16) | yLow
                # angleZ = (zHigh << 16) | zLow

                angleX = parse32(0)
                angleY = parse32(4)
                angleZ = parse32(8)

                self.angleData = np.array([angleX, angleY, angleZ])
            else:
                if len(data) != 6:
                    return

                values = struct.unpack(">hhh", data)
                if addr == 0x34:
                    self.accData = np.array(values)
                elif addr == 0x37:
                    self.gyroData = np.array(values)
                elif addr == 0x3A:
                    self.magData = np.array(values)

        self.publishImu()

        self.waitingResponse = False
        self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)

    # ==========================================================
    # 发布IMU
    # ==========================================================
    def publishImu(self):

        accScale = 16.0 / 32768.0
        gyroScale = 2000.0 / 32768.0

        ax, ay, az = self.accData * accScale
        gx, gy, gz = np.radians(self.gyroData * gyroScale)
        mx, my, mz = self.magData

        if self.protocol in (
            protocolType.TTL_HIGH,
            protocolType.CAN_HIGH,
            protocolType.RS485_HIGH,
        ):
            roll, pitch, yaw = np.radians(self.angleData / 1000.0)
        else:
            angleScale = 180.0 / 32768.0
            roll, pitch, yaw = np.radians(self.angleData * angleScale)

        self.acc = (ax, ay, az)
        self.gyro = (gx, gy, gz)
        self.angle = (roll, pitch, yaw)
        self.mag = (mx, my, mz)

        quaternion = self.eulerToQuaternion(roll, pitch, yaw)

        imuMsg = Imu()
        imuMsg.header.stamp = self.get_clock().now().to_msg()
        imuMsg.header.frame_id = "imu_link"

        imuMsg.linear_acceleration.x = float(ax)
        imuMsg.linear_acceleration.y = float(ay)
        imuMsg.linear_acceleration.z = float(az)

        imuMsg.angular_velocity.x = float(gx)
        imuMsg.angular_velocity.y = float(gy)
        imuMsg.angular_velocity.z = float(gz)

        imuMsg.orientation.x = quaternion[0]
        imuMsg.orientation.y = quaternion[1]
        imuMsg.orientation.z = quaternion[2]
        imuMsg.orientation.w = quaternion[3]

        self.imuPublisher.publish(imuMsg)
        if getattr(self, "publish_mag", False) and getattr(self, "magPublisher", None):
            magMsg = MagneticField()
            magMsg.header = imuMsg.header
            magMsg.magnetic_field.x = float(mx)
            magMsg.magnetic_field.y = float(my)
            magMsg.magnetic_field.z = float(mz)
            magMsg.magnetic_field_covariance = [0.0] * 9
            self.magPublisher.publish(magMsg)

        imuRpyMsg = ImuData()
        imuRpyMsg.header = imuMsg.header
        imuRpyMsg.imu = imuMsg
        imuRpyMsg.roll = math.degrees(roll)
        imuRpyMsg.pitch = math.degrees(pitch)
        imuRpyMsg.yaw = math.degrees(yaw)

        self.imuRpyPublisher.publish(imuRpyMsg)
        # 将 IMU 数据按行加入 CSV 缓冲（主线程只做构建与入队，写入在后台线程）
        if getattr(self, "_csv_enabled", False):
            try:
                stamp = imuMsg.header.stamp
                stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
                line = (
                    f"{stamp_ns},{float(ax):.6f},{float(ay):.6f},{float(az):.6f},"
                    f"{float(gx):.9f},{float(gy):.9f},{float(gz):.9f},"
                    f"{math.degrees(roll):.6f},{math.degrees(pitch):.6f},{math.degrees(yaw):.6f},"
                    f"{float(mx):.6f},{float(my):.6f},{float(mz):.6f}\n"
                )
                with self._csv_lock:
                    self._csv_buffer.append(line)
                    if len(self._csv_buffer) >= self._csv_buffer_size:
                        # 通知写线程立即写入
                        try:
                            self._csv_event.set()
                        except Exception:
                            pass
            except Exception as e:
                try:
                    self.get_logger().error(f"CSV buffer error: {e}")
                except Exception:
                    print(f"CSV buffer error: {e}")
        # 更新时间戳，表示最近一次成功发布 IMU 数据（用于 watchdog）
        try:
            self.last_success_ts = self.get_clock().now().nanoseconds
        except Exception:
            # 兜底：使用系统时间（纳秒）
            self.last_success_ts = int(time.time() * 1e9)

    # ==========================================================
    # 工具函数
    # ==========================================================
    def buildModbusReadCmd(self, addr, length):
        frame = bytearray()
        frame.append(self.modbusID)
        frame.append(0x03)
        frame.append((addr >> 8) & 0xFF)
        frame.append(addr & 0xFF)
        frame.append((length >> 8) & 0xFF)
        frame.append(length & 0xFF)

        crc = self.modbusCRC(frame)
        frame.append(crc & 0xFF)  # CRC_L
        frame.append((crc >> 8) & 0xFF)  # CRC_H

        return bytes(frame)

    @staticmethod
    def ttlChecksum(frame):
        return (sum(frame[0:10]) & 0xFF) == frame[10]

    @staticmethod
    def modbusCRC(data: bytes):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def eulerToQuaternion(roll, pitch, yaw):

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return [qx, qy, qz, qw]

    def printMsg(self):
        print(
            f"Accel: x={self.acc[0]:+12.3f} | y={self.acc[1]:+12.3f} | z={self.acc[2]:+12.3f} m/s²"
        )
        print(
            f"Gyro:  x={math.degrees(self.gyro[0]):+12.3f} | y={math.degrees(self.gyro[1]):+12.3f} | z={math.degrees(self.gyro[2]):+12.3f} °/s"
        )
        print(
            f"Angle: x={math.degrees(self.angle[0]):+12.3f} | y={math.degrees(self.angle[1]):+12.3f} | z={math.degrees(self.angle[2]):+12.3f} °"
        )
        print(
            f"Mag:   x={self.mag[0]:+12.3f} | y={self.mag[1]:+12.3f} | z={self.mag[2]:+12.3f}"
        )
        print("\033[H\033[J", end="")  # 清屏

    # 后台 CSV 写线程
    def _csv_writer_thread(self):
        # 等待事件或定时唤醒，批量写入缓冲
        try:
            while not getattr(self, "_csv_stop_event", threading.Event()).is_set():
                try:
                    self._csv_event.wait(timeout=self._csv_flush_interval_s)
                except Exception:
                    pass

                # 提取缓冲内容
                lines = None
                with self._csv_lock:
                    if self._csv_buffer:
                        lines = "".join(self._csv_buffer)
                        self._csv_buffer.clear()
                    # 清除事件以便下一次等待
                    try:
                        self._csv_event.clear()
                    except Exception:
                        pass

                if lines:
                    try:
                        if self._csv_file:
                            self._csv_file.write(lines)
                            self._csv_file.flush()
                    except Exception as e:
                        try:
                            self.get_logger().error(f"CSV write failed: {e}")
                        except Exception:
                            print(f"CSV write failed: {e}")
                        # 出错则禁用后续记录
                        self._csv_enabled = False
                        break
        finally:
            # 退出前尝试 flush 剩余数据
            try:
                with self._csv_lock:
                    if getattr(self, "_csv_buffer", None) and self._csv_file:
                        self._csv_file.write("".join(self._csv_buffer))
                        self._csv_buffer.clear()
                        self._csv_file.flush()
            except Exception:
                pass

    # ==========================================================
    # Watchdog 与 lifecycle 触发
    # ==========================================================
    def _call_deactivate_and_log(self, target):
        try:
            cmd = ["ros2", "lifecycle", "set", target, "deactivate"]
            print(f"Calling lifecycle CLI: {' '.join(cmd)}", flush=True)
            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=15,
            )
            out = proc.stdout.strip()
            err = proc.stderr.strip()
            if out:
                print(f"ros2 lifecycle stdout: {out}", flush=True)
            if proc.returncode != 0:
                print(f"ros2 lifecycle failed: {err}", flush=True)
        except Exception as e:
            print(f"Exception calling ros2 lifecycle: {e}", flush=True)

    def watchdogCallback(self):
        # 已经报告过则忽略
        if getattr(self, "_fault_reported", False):
            return

        now = self.get_clock().now().nanoseconds
        elapsed_ns = now - getattr(self, "last_success_ts", now)
        if elapsed_ns > int(self.imu_watchdog_timeout * 1e9):
            # 触发故障处理
            self.get_logger().error(
                f"IMU watchdog triggered: no valid IMU publish for {self.imu_watchdog_timeout}s"
            )
            self._fault_reported = True

            if self.deactivate_zmotion_on_fault:
                ns = (
                    str(self.namespace_param).strip("/") if self.namespace_param else ""
                )
                if ns:
                    target = f"/{ns}/{self.zmotion_driver_node_name}"
                else:
                    target = self.zmotion_driver_node_name

                # 同步调用 lifecycle CLI，避免在 rclpy.shutdown() 后使用 ROS logger 的 race
                try:
                    self._call_deactivate_and_log(target)
                except Exception as e:
                    print(f"Exception in lifecycle call: {e}", flush=True)

            # 请求 ROS 事件循环退出，从而让进程优雅终止
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def shutdown(self):
        if self.serialPort and self.serialPort.is_open:
            self.serialPort.close()
            self.get_logger().info("Serial port closed")
        # 停止 CSV 写线程并 flush/关闭文件
        if getattr(self, "_csv_thread", None):
            try:
                self._csv_stop_event.set()
                try:
                    self._csv_event.set()
                except Exception:
                    pass
                # 等待写线程退出
                try:
                    self._csv_thread.join(timeout=2.0)
                except Exception:
                    pass
                if getattr(self, "_csv_file", None):
                    try:
                        self._csv_file.close()
                    except Exception:
                        pass
                try:
                    self.get_logger().info("CSV writer stopped and file closed")
                except Exception:
                    pass
            except Exception as e:
                try:
                    self.get_logger().error(f"Error shutting down CSV writer: {e}")
                except Exception:
                    print(f"Error shutting down CSV writer: {e}")


def main():

    rclpy.init()
    node = imuDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()  # 关闭串口
        rclpy.spin_once(node, timeout_sec=0.1)  # 确保回调退出
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
