#ifndef BROVER_SYSTEM_H
#define BROVER_SYSTEM_H

#include <map>
#include <iostream>

#include <boost/format.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cyphal_ros2_bridge/msg/drive_simple_state.hpp>

#include "hardware_interface/system_interface.hpp"

namespace brover_hardware {

using namespace hardware_interface;

using cb_return_type = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using LifecycleState = rclcpp_lifecycle::State;
using std::placeholders::_1;

class WheelHardware {
protected:
    std::string this_node_name;

    rclcpp::Logger logger;
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub;
    rclcpp::Subscription<cyphal_ros2_bridge::msg::DriveSimpleState>::SharedPtr drv_state_sub;

    float angle_offset = NAN;
    float angle = 0;
    float real_velocity = 0;
    float target_velocity = 0;
public:
    WheelHardware(rclcpp::Node::SharedPtr& node, const std::string& node_name)
        : this_node_name(node_name)
        , logger(node->get_logger())
        , clock(node->get_clock())
        {
        velocity_pub = node->create_publisher<std_msgs::msg::Float32>(
            (boost::format("/%1%/angular_velocity") % node_name).str(),
            10
        );
        drv_state_sub = node->create_subscription<cyphal_ros2_bridge::msg::DriveSimpleState>(
            (boost::format("/%1%/drive_state") % node_name).str(),
            10,
            std::bind(&WheelHardware::drv_state_cb, this, _1)
        );
    }

    void drv_state_cb(const cyphal_ros2_bridge::msg::DriveSimpleState::SharedPtr msg) {
        if (std::isnan(angle_offset)) {
            angle_offset = msg->angle;
        }
        angle = msg->angle;
        real_velocity = msg->velocity;
    }

    void set_target(float vel) {
        RCLCPP_INFO_THROTTLE(
            logger,
            *clock,
            250,
            (boost::format("%1%/velocity_cmd %2%") % this_node_name % vel).str().c_str()
        );
        target_velocity = vel;
    }

    void send_command() {
        auto msg = std_msgs::msg::Float32();
        msg.data = target_velocity;
        velocity_pub->publish(msg);
    }

    float get_angle() const {
        return angle - angle_offset;
    }

    float get_velocity() const {
        return real_velocity;
    }
};

class BroverSystemHardware: public SystemInterface {
protected:
    std::map<std::string, std::shared_ptr<WheelHardware>> wheels;
    rclcpp::TimerBase::SharedPtr send_cmd_timer;
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BroverSystemHardware)

    cb_return_type on_configure(const LifecycleState& previous_state) override;
    cb_return_type on_cleanup(const LifecycleState& previous_state) override;
    cb_return_type on_shutdown(const LifecycleState& previous_state) override;
    cb_return_type on_activate(const LifecycleState& previous_state) override;
    cb_return_type on_deactivate(const LifecycleState& previous_state) override;
    cb_return_type on_error(const LifecycleState& previous_state) override;
    cb_return_type on_init(const HardwareComponentInterfaceParams& params) override;

    std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
    std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    void send_command_cb();
};

}
#endif
