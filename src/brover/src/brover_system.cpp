#include "brover/brover_system.h"

#include <chrono>

using namespace std::chrono_literals;

namespace brover_hardware {

#pragma region lifecycle
cb_return_type BroverSystemHardware::on_configure(const LifecycleState& previous_state) {
    for (const auto & [name, descr] : joint_state_interfaces_){
        set_state(name, 0.0);
    }
    for (const auto & [name, descr] : joint_command_interfaces_) {
        set_command(name, 0.0);
    }

    if (auto node = get_node()) {
        wheels["lf_wheel_joint"] = std::make_shared<WheelHardware>(node, "node1");
        wheels["lr_wheel_joint"] = std::make_shared<WheelHardware>(node, "node2");
        wheels["rf_wheel_joint"] = std::make_shared<WheelHardware>(node, "node3");
        wheels["rr_wheel_joint"] = std::make_shared<WheelHardware>(node, "node4");

        send_cmd_timer = node->create_wall_timer(
            10ms,
            std::bind(&BroverSystemHardware::send_command_cb, this)
        );
    }
    else {
        RCLCPP_FATAL(get_logger(), "ROS Node failed to initialize");
        rclcpp::shutdown();
    }
    return cb_return_type::SUCCESS;
};

cb_return_type BroverSystemHardware::on_cleanup(const LifecycleState& previous_state) {
    return SystemInterface::on_cleanup(previous_state);
};

cb_return_type BroverSystemHardware::on_shutdown(const LifecycleState& previous_state) {
    return SystemInterface::on_shutdown(previous_state);
};

cb_return_type BroverSystemHardware::on_activate(const LifecycleState& previous_state) {
    for (const auto & [name, descr] : joint_command_interfaces_) {
        set_command(name, get_state(name));
    }
    return SystemInterface::on_activate(previous_state);
};

cb_return_type BroverSystemHardware::on_deactivate(const LifecycleState& previous_state) {
    return SystemInterface::on_deactivate(previous_state);
};

cb_return_type BroverSystemHardware::on_error(const LifecycleState& previous_state) {
    return SystemInterface::on_error(previous_state);
};

cb_return_type BroverSystemHardware::on_init(const HardwareComponentInterfaceParams& params) {
    return SystemInterface::on_init(params);
};

std::vector<StateInterface::ConstSharedPtr> BroverSystemHardware::on_export_state_interfaces() {
    return SystemInterface::on_export_state_interfaces();
}

std::vector<CommandInterface::SharedPtr> BroverSystemHardware::on_export_command_interfaces() {
    return SystemInterface::on_export_command_interfaces();
}
#pragma endregion

void BroverSystemHardware::send_command_cb() {
    for (auto& [node_name, wheel_hardware] : wheels) {
        wheel_hardware->send_command();
    }
}

return_type BroverSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    for (const auto& [name, descr] : joint_state_interfaces_) {
        auto wheel_name = descr.get_prefix_name();
        auto wheel_hardware = wheels[wheel_name];
        if (!wheel_hardware) {
            RCLCPP_ERROR(
                get_logger(),
                (boost::format("Required wheel <%1%> does not exist") % wheel_name).str().c_str()
            );
            continue;
        }

        if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION) {
            set_state<double>(name, wheel_hardware->get_angle());
        }
        if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
            set_state<double>(name, wheel_hardware->get_velocity());
        }
    }

    return hardware_interface::return_type::OK;
}

return_type BroverSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    for (const auto& [name, descr] : joint_command_interfaces_) {
        auto wheel_name = descr.get_prefix_name();
        auto wheel_hardware = wheels[wheel_name];
        if (!wheel_hardware) {
            RCLCPP_ERROR(
                get_logger(),
                (boost::format("Required wheel <%1%> does not exist") % wheel_name).str().c_str()
            );
            get_command(name);
            continue;
        }

        wheel_hardware->set_target(get_command(name));
    }

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(brover_hardware::BroverSystemHardware, hardware_interface::SystemInterface)
