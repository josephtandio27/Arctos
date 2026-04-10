#ifndef ARCTOS_SERIAL_HARDWARE_INTERFACE_HPP_
#define ARCTOS_SERIAL_HARDWARE_INTERFACE_HPP_

#include <thread>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <boost/asio.hpp> // For serial communication

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_component_interface.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/string.hpp>

using hardware_interface::return_type;

namespace arctos_control
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class ArctosSerialHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        // Core Read/Write loops
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Interface setup
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        // Node for live parameters update
        std::atomic<bool> thread_running_{false};
        std::thread service_thread_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::mutex serial_mutex_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr feed_rate_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grbl_config_sub_;

        // Storage for joint values (ROS works in Radians)
        std::vector<double> hw_commands_;
        std::vector<double> hw_states_;
        std::vector<double> last_sent_commands_;
        double gripper_velocity_state_ = 0.0;
        double last_gripper_position_ = 0.0;

        // Serial communication placeholder
        boost::asio::io_service io_service_;
        std::unique_ptr<boost::asio::serial_port> serial_port_;
        boost::asio::streambuf read_buffer_;
        std::string port_name_;
        int port_baud_rate_;

        // Motion parameters
        std::atomic<int> feed_rate_;
        std::string motion_type_gcode_; // "G90" (absolute) or "G91" (relative)

        // Token Logic
        std::atomic<bool> arduino_ready_{true};

        void async_read_ok();
        void setup_subscribers();
    };
}

#endif // ARCTOS_SERIAL_HARDWARE_INTERFACE_HPP_