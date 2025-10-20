#include "motor_hw/motor_interface.h"
#include "motor_hw/motor_hw.h"
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn motor_interface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    // 根据关节数量动态初始化命令和状态变量
    const size_t joint_count = info.joints.size();
    joint_torque_command_.assign(joint_count, 0);
    joint_position_command_.assign(joint_count, 0);
    joint_velocities_command_.assign(joint_count, 0);
    joint_kp_command_.assign(joint_count, 0);
    joint_kd_command_.assign(joint_count, 0);

    joint_position_.assign(joint_count, 0);
    joint_velocities_.assign(joint_count, 0);
    joint_effort_.assign(joint_count, 0);
    // 解析hardware_info，获取关节名称与接口的对应关系
    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    // 验证每个关节的接口配置
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly在每个关节上有5个命令接口
    if (joint.command_interfaces.size() != 5)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("motor_interface"), "Joint '%s' has %zu command interfaces found. 5 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // RRBotSystemPositionOnly在每个关节上有3个状态接口
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("motor_interface"), "Joint '%s' has %zu state interfaces found. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }


    //串口号，电机canid masterid、电机类型等参数在这里解析
     std::string yaml_file_path = "";
    if (info_.hardware_parameters.find("yaml_file_path") != info_.hardware_parameters.end()) {
        yaml_file_path = info_.hardware_parameters.at("yaml_file_path");
    } 
    
    // 解析电机配置
    if (!parseDmActData(yaml_file_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), 
            "Failed to parse motor configuration from %s", yaml_file_path.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;

}


std::vector<hardware_interface::StateInterface> motor_interface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> motor_interface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;

}




bool motor_interface::parseDmActData(const std::string& yaml_file_path)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        
        if (!config["motor_ports"] || !config["motors"]) {
            RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), 
                "YAML file missing required 'motor_ports' or 'motors' section");
            return false;
        }
        // 打印配置文件路径
        RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
            "Parsing file: %s", yaml_file_path.c_str());
        
        // 解析 motors 配置
        const auto& motors = config["motors"];
        if (motors.IsSequence()) {
            for (const auto& motor : motors) {
                std::string port = motor["port"].as<std::string>();
                int can_id = motor["can_id"].as<int>();
                int master_id = motor["master_id"].as<int>();
                std::string name = motor["name"].as<std::string>();
                std::string type_str = motor["type"].as<std::string>();
                
                // 转换电机类型字符串为枚举
                damiao::DM_Motor_Type motor_type;
                if (type_str == "DM4310") motor_type = damiao::DM4310;
                else if (type_str == "DM4310_48V") motor_type = damiao::DM4310_48V;
                else if (type_str == "DM4340") motor_type = damiao::DM4340;
                else if (type_str == "DM4340_48V") motor_type = damiao::DM4340_48V;
                else if (type_str == "DM6006") motor_type = damiao::DM6006;
                else if (type_str == "DM8006") motor_type = damiao::DM8006;
                else if (type_str == "DM8009") motor_type = damiao::DM8009;
                else if (type_str == "DM10010L") motor_type = damiao::DM10010L;
                else if (type_str == "DM10010") motor_type = damiao::DM10010;
                else if (type_str == "DMH3510") motor_type = damiao::DMH3510;
                else if (type_str == "DMH6215") motor_type = damiao::DMH6215;
                else if (type_str == "DMG6220") motor_type = damiao::DMG6220;
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), 
                        "Unknown motor type: %s", type_str.c_str());
                    return false;
                }
                
                // 创建 DmActData 结构
                damiao::DmActData dm_data;
                dm_data.name = name;
                dm_data.motorType = motor_type;
                dm_data.can_id = can_id;
                dm_data.mst_id = master_id;
                
                // 将数据添加到 port_id2dm_data_ 映射中
                port_id2dm_data_[port][can_id] = dm_data;
                RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
                    "Added motor: %s (%s, CAN ID: %d, Master ID: %d)", 
                    name.c_str(), type_str.c_str(), can_id, master_id);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
            "Successfully parsed motor configuration from %s", yaml_file_path.c_str());
        // 解析 motor_ports 配置
        const auto& ports = config["motor_ports"];
        if (ports.IsSequence()) {
            motor_ports_.clear();
            motor_ports_.reserve(ports.size());
            RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
                "Found %d motor ports in configuration", ports.size());
            for (const auto& port : ports) {
                std::string port_name = port["port"].as<std::string>();
                int baudrate = port["baudrate"].as<int>();
                RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
                    "Adding motor port: %s (Baudrate: %d)", port_name.c_str(), baudrate);
                // 创建 Motor_Control 实例
                auto motor_control = std::make_shared<damiao::Motor_Control>(
                    port_name, baudrate, &port_id2dm_data_[port_name]);
                    RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
                        "Added motor port: %s (Baudrate: %d)", port_name.c_str(), baudrate);
                motor_ports_.push_back(motor_control);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("motor_interface"), 
            "Successfully parsed motor configuration from %s", yaml_file_path.c_str());
        return true;
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), 
            "Failed to parse YAML file %s: %s", yaml_file_path.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_interface"), 
            "Error while parsing motor configuration: %s", e.what());
        return false;
    }
}


return_type motor_interface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
   
    int ind = 0;
    
    // 处理position命令接口
    for (const auto& joint_name : joint_interfaces["position"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    can_entry.second.cmd_pos = joint_position_command_[ind];
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    ind = 0;
    // 处理velocity命令接口
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    //can_entry.second.cmd_vel = joint_velocities_command_[ind];
                    can_entry.second.cmd_vel = 0.1;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    ind = 0;
    // 处理effort命令接口
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    can_entry.second.cmd_effort = joint_torque_command_[ind];
                    //can_entry.second.cmd_effort = 1.0;
                    //can_entry.second.kp = joint_kp_command_[ind];
                    can_entry.second.kp = 1;
                    can_entry.second.kd = joint_kd_command_[ind];
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    // 2. 调用每个Motor_Control实例的write()方法，将命令发送到硬件
    for (auto& motor_control : motor_ports_)
    {
        motor_control->write();
    }
    
    return return_type::OK;
}


return_type motor_interface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // 1. 调用每个Motor_Control实例的read()方法，从硬件读取状态
    //    该操作会自动更新port_id2dm_data_中的pos/vel/effort字段
    for (auto& motor_control : motor_ports_)
    {
        motor_control->read();
    }
    
    // 2. 从port_id2dm_data_中提取状态值并填充到ROS2接口
    int ind = 0;
    
    // 处理position状态接口
    for (const auto& joint_name : joint_interfaces["position"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    joint_position_[ind] = can_entry.second.pos;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    ind = 0;
    // 处理velocity状态接口
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    joint_velocities_[ind] = can_entry.second.vel;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    ind = 0;
    // 处理effort状态接口
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        bool found = false;
        for (auto& port_entry : port_id2dm_data_)
        {
            for (auto& can_entry : port_entry.second)
            {
                if (can_entry.second.name == joint_name)
                {
                    joint_effort_[ind] = can_entry.second.effort;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (!found) {
            RCLCPP_WARN(rclcpp::get_logger("motor_interface"), 
                "Joint %s not found in port_id2dm_data_", joint_name.c_str());
        }
        ind++;
    }
    
    return return_type::OK;
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motor_interface,
  hardware_interface::SystemInterface)





