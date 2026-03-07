#include <iostream>
#include <memory>

#include "zmc432_driver/motion_controller.hpp"

int main()
{
  std::cout << "测试电机使能功能..." << std::endl;

  // 创建运动控制器
  auto controller = std::make_unique<zmc432_driver::MotionController>();

  // 初始化控制器（使用模拟IP地址）
  auto init_result = controller->initialize("192.168.0.11");
  if (init_result)
  {
    std::cout << "连接失败: " << init_result.value() << std::endl;
    return 1;
  }

  std::cout << "控制器连接成功" << std::endl;

  // 配置轴0
  auto config_result = controller->configure_axis(0, 1.0, 10.0, 100.0, 100.0);
  if (config_result)
  {
    std::cout << "轴配置失败: " << config_result.value() << std::endl;
    return 1;
  }

  std::cout << "轴0配置成功" << std::endl;

  // 测试使能轴0
  auto enable_result = controller->enable_axis(0, true);
  if (enable_result)
  {
    std::cout << "轴使能失败: " << enable_result.value() << std::endl;
    return 1;
  }

  std::cout << "轴0使能成功" << std::endl;

  // 读取使能状态
  auto enable_status = controller->get_axis_enable(0);
  if (enable_status)
  {
    std::cout << "轴0使能状态: "
              << (enable_status.value() ? "已使能" : "未使能") << std::endl;
  }
  else
  {
    std::cout << "无法读取轴0使能状态" << std::endl;
  }

  // 测试失能轴0
  auto disable_result = controller->enable_axis(0, false);
  if (disable_result)
  {
    std::cout << "轴失能失败: " << disable_result.value() << std::endl;
    return 1;
  }

  std::cout << "轴0失能成功" << std::endl;

  // 再次读取使能状态
  enable_status = controller->get_axis_enable(0);
  if (enable_status)
  {
    std::cout << "轴0使能状态: "
              << (enable_status.value() ? "已使能" : "未使能") << std::endl;
  }
  else
  {
    std::cout << "无法读取轴0使能状态" << std::endl;
  }

  std::cout << "使能功能测试完成" << std::endl;
  return 0;
}
