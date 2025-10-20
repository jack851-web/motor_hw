#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H
#include "motor_hw/motor_hw.h"
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/handle.hpp"                     // 硬件接口句柄
#include "hardware_interface/hardware_info.hpp"              // 硬件信息结构
#include "hardware_interface/system_interface.hpp"           // 系统接口基类
#include "hardware_interface/types/hardware_interface_return_values.hpp"  // 接口返回值类型
#include "rclcpp/clock.hpp"                                  // ROS2时钟
#include "rclcpp/logger.hpp"                                 // ROS2日志记录器
#include "rclcpp/macros.hpp"                                 // ROS2宏定义
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"  // 生命周期节点接口
#include "rclcpp_lifecycle/state.hpp"                        // 生命周期状态

class motor_interface final : public hardware_interface::SystemInterface{

public:

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    bool parseDmActData(const std::string& yaml_file_path);
    

    protected:
    
    // 关节控制命令
    std::vector<double> joint_torque_command_;      // 力矩指令
    std::vector<double> joint_position_command_;    // 位置指令
    std::vector<double> joint_velocities_command_;  // 速度指令
    std::vector<double> joint_kp_command_;          // 位置增益
    std::vector<double> joint_kd_command_;          // 速度增益

    // 关节状态反馈
    std::vector<double> joint_position_;    // 当前位置
    std::vector<double> joint_velocities_;  // 当前速度
    std::vector<double> joint_effort_;      // 当前力矩
    //记录关节名称与接口的对应关系
    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };

     // 每个串口对应一个元素
    std::vector<std::shared_ptr<damiao::Motor_Control>> motor_ports_{};
  
    // 通过端口名称和CAN、MST ID找到对应的关节电机名称
    std::unordered_map<std::string,  std::unordered_map<int,damiao::DmActData>> port_id2dm_data_{};



};

#endif//motor_interface




