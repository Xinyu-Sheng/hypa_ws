#include <iostream>
#include <memory>
#include "zmc432_driver/motion_controller.hpp"
#include "zmc432_driver/ecat_init.hpp"

int main(int argc, char** argv)
{
  std::cout << "测试 EtherCAT 总线初始化..." << std::endl;

  auto controller = std::make_unique<zmc432_driver::MotionController>();
  std::string ip = "192.168.0.11";
  if (argc >= 2) {
    ip = argv[1];
  }

  auto init_result = controller->initialize(ip);
  if (init_result) {
    std::cout << "连接失败: " << init_result.value() << std::endl;
    return 1;
  }

  zmc432_driver::EcatInitInfo info;
  // 默认配置

  auto ecerr = controller->initialize_bus(info);
  if (ecerr) {
    std::cout << "EtherCAT 初始化失败: " << ecerr.value() << std::endl;
    return 1;
  }

  std::cout << "EtherCAT 初始化成功" << std::endl;
  return 0;
}
