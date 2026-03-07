#include <iostream>
#include <memory>
#include "zmc432_driver/motion_controller.hpp"
#include "zmc432_driver/ecat_init.hpp"

int main(int argc, char** argv)
{
  std::cout << "测试 EtherCAT 总线初始化..." << std::endl;

  auto controller = std::make_unique<zmc432_driver::MotionController>();
  std::string ip = "192.168.0.11";
  if (argc >= 2)
  {
    ip = argv[1];
  }

  auto init_result = controller->initialize(ip);
  if (init_result)
  {
    std::cout << "连接失败: " << init_result.value() << std::endl;
    return 1;
  }

  zmc432_driver::EcatInitInfo info;
  // 默认配置

  auto ecerr = controller->initialize_bus(info);
  if (ecerr)
  {
    std::cout << "EtherCAT 初始化失败: " << ecerr.value() << std::endl;
    return 1;
  }
  std::cout << "EtherCAT 初始化成功" << std::endl;

  // 演示修改映射参数后再次调用（适用于硬件有多个从站时）
  info.drive_pdo_mode[0] = 4;  // 将第一个轴设置为某模式
  info.drive_io_stara = 512;
  info.drive_io_spa = 8;
  info.drive_enable = 1;
  info.drive_axis_num = -1;  // 不验证轴数目
  info.ecat_node_num = -1;   // 不验证节点数目

  auto ecerr2 = controller->initialize_bus(info);
  if (ecerr2)
  {
    std::cout << "二维配置初始化失败: " << ecerr2.value() << std::endl;
  }
  else
  {
    std::cout << "二维配置初始化调用成功（硬件应反映映射/使能设置）"
              << std::endl;
  }
  return 0;
}
