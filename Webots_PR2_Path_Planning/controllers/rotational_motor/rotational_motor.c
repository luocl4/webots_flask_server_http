#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64  // 与Webots世界文件中的基本时间步一致

int main(int argc, char **argv) {
  // 初始化Webots API
  wb_robot_init();
  
  // 获取电机设备（名称需与.wbt文件中一致）
  WbDeviceTag motor = wb_robot_get_device("motor");
  
  // 禁用位置控制，启用速度控制模式
  wb_motor_set_position(motor, INFINITY);
  
  // 设置目标速度（弧度/秒）
  const double target_velocity = 0.2;  // 可根据需要调整
  wb_motor_set_velocity(motor, target_velocity);
  
  // 主循环：保持电机持续旋转
  while (wb_robot_step(TIME_STEP) != -1) {
    // 无需额外操作，电机将持续以设定速度旋转
  }
  
  // 清理并退出
  wb_robot_cleanup();
  return 0;
}