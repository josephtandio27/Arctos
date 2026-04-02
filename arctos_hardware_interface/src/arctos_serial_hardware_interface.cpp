#include "arctos_hardware_interface/arctos_serial_hardware_interface.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

namespace arctos_control
{

    CallbackReturn ArctosSerialHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        auto logger = rclcpp::get_logger("ArctosHardware");
        node_ = std::make_shared<rclcpp::Node>(params.hardware_info.name + "_node");
        feed_rate_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "set_feed_rate",
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->feed_rate_.store(msg->data);
                RCLCPP_INFO(node_->get_logger(), "Feed rate updated to: %d", this->feed_rate_.load());
            });

        // Initialize vectors for 7 joints
        hw_commands_.resize(params.hardware_info.joints.size(), 0.0);
        hw_states_.resize(params.hardware_info.joints.size(), 0.0);

        // Get the port name from the URDF parameters
        port_name_ = params.hardware_info.hardware_parameters.at("serial_port");
        port_baud_rate_ = std::stoi(params.hardware_info.hardware_parameters.at("baud_rate"));

        // Read feed rate (Default to 800 if not found)
        if (params.hardware_info.hardware_parameters.find("feed_rate") != params.hardware_info.hardware_parameters.end())
        {
            feed_rate_ = std::stoi(params.hardware_info.hardware_parameters.find("feed_rate")->second);
        }
        else
        {
            feed_rate_ = 800;
        }

        // Read coordinate mode
        if (params.hardware_info.hardware_parameters.at("abs_motion") == "true")
        {
            motion_type_gcode_ = "G90";
        }
        else
        {
            motion_type_gcode_ = "G91";
        }
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Configured to use port: %s, with feed rate: %d, motion type: %s", port_name_.c_str(), feed_rate_.load(), motion_type_gcode_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            // Allocate the object memory
            serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_);

            // We don't open it yet, just prepare it.
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Serial port object created.");
        }
        catch (const std::exception &e)
        {
            return CallbackReturn::ERROR;
        }

        // // reset values always when configuring hardware
        // for (const auto &[name, descr] : joint_state_interfaces_)
        // {
        //     set_state(name, 0.0);
        // }
        // for (const auto &[name, descr] : joint_command_interfaces_)
        // {
        //     set_command(name, 0.0);
        // }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            // Now we actually try to talk to the hardware
            serial_port_->open(port_name_); // Or your udev symlink
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(port_baud_rate_));

            // Initialize/Reset the movement tracker to the current commands
            last_sent_commands_.assign(hw_commands_.size(), 0.0);

            // Start your "ok" token listener
            arduino_ready_.store(true);
            async_read_ok();

            RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Arctos Connected!");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardware"), "Activation failed: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            arduino_ready_ = false;
            if (serial_port_->is_open())
            {
                serial_port_->close();
            }
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Arctos Disconnected.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardware"), "Deactivation failed: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Cleaning up Arctos serial resources...");

        try
        {
            // 1. Stop the Boost IO service
            // This cancels any pending async_read_until ("ok" listener)
            if (!io_service_.stopped())
            {
                io_service_.stop();
            }

            // 2. Close and destroy the serial port object
            if (serial_port_)
            {
                if (serial_port_->is_open())
                {
                    serial_port_->close();
                }
                // Resetting the unique_ptr releases the memory allocated in on_configure
                serial_port_.reset();
            }

            RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Serial port destroyed.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardware"), "Error during cleanup: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // 3. Optional: Clear the internal maps/vectors if you want a true "fresh start"
        // Though ros2_control usually handles the interface lifecycle,
        // zeroing them out again matches your on_configure logic.
        for (auto const &[name, descr] : joint_state_interfaces_)
        {
            set_state(name, 0.0);
        }
        for (auto const &[name, descr] : joint_command_interfaces_)
        {
            set_command(name, 0.0);
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "System shutting down. Closing Arctos connection...");

        try
        {
            // 1. Final Hardware Command (Safety)
            // If the port is open, try one last quick write to disable motors
            if (serial_port_ && serial_port_->is_open())
            {
                // M84 usually disables steppers in GRBL/Marlin
                // We use a synchronous write here because we need it done NOW.
                boost::asio::write(*serial_port_, boost::asio::buffer("M84\n"));

                // 2. Close the port
                serial_port_->close();
            }

            // 3. Stop the IO service
            if (!io_service_.stopped())
            {
                io_service_.stop();
            }

            // 4. Release memory
            serial_port_.reset();
        }
        catch (...)
        {
            // In on_shutdown, we don't want to throw or crash further.
            // We catch everything and just let the node die quietly.
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArctosSerialHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // Add a log to see exactly what is happening during the crash
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Exporting state interface for: %s", info_.joints[i].name.c_str());
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArctosSerialHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type ArctosSerialHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // OPEN LOOP: Since we have no encoders, we assume the robot reached the command.
        for (uint i = 0; i < hw_states_.size(); i++)
        {
            hw_states_[i] = hw_commands_[i];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArctosSerialHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Process ROS callback to update feed_rate_
        if (node_)
            rclcpp::spin_some(node_);

        // Allow boost to process the 'ok' callbacks
        io_service_.poll();
        io_service_.reset();

        // Check if joints actually moved enough to justify a new G-code command
        if (last_sent_commands_.size() != hw_commands_.size())
        {
            return hardware_interface::return_type::OK;
        }
        bool should_move = false;
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            if (std::abs(hw_commands_[i] - last_sent_commands_[i]) > 0.001)
            { // ~0.05 degrees
                should_move = true;
                break;
            }
        }

        // 1. Only send if Arduino is ready (Token-based) AND if the command has actually changed
        // Note: In open loop, sending the same G-code 100 times a second will clog the buffer.
        if (arduino_ready_.load() && should_move)
        {
            last_sent_commands_ = hw_commands_; // Update last sent commands
            // 2. Format the G-code string
            // G1 = Coordinated Linear Move
            // F800 = Feed rate in mm/min (or deg/min depending on your GRBL setup)
            // G90 = Absolute positioning
            std::stringstream ss;
            ss << "G1 F" << feed_rate_.load() << " " << motion_type_gcode_;
            char axes[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};

            // Loop through the 6 joints provided by hw_commands_
            for (size_t i = 0; i < hw_commands_.size() && i < 6; i++)
            {
                // ROS 2 provides Radians; Arctos GRBL expects Degrees
                double degrees = hw_commands_[i] * (180.0 / M_PI);
                // Format to 2 decimal places for GRBL precision
                ss << " " << axes[i] << std::fixed << std::setprecision(2) << degrees;
            }
            ss << "\n";

            std::string gcode_cmd = ss.str();

            // 3. Consume the token
            arduino_ready_ = false;

            // 4. Send via Boost.Asio
            boost::asio::async_write(*serial_port_, boost::asio::buffer(gcode_cmd),
                                     [this](const boost::system::error_code &ec, std::size_t /*bytes_transferred*/)
                                     {
                                         if (ec)
                                         {
                                             RCLCPP_ERROR(rclcpp::get_logger("ArctosHardware"), "Write failed: %s", ec.message().c_str());
                                             // If write fails, we might want to reset the token so we can try again
                                             arduino_ready_ = true;
                                         }
                                     });
        }

        return hardware_interface::return_type::OK;
    }

    void ArctosSerialHardwareInterface::async_read_ok()
    {
        if (!serial_port_ || !serial_port_->is_open())
            return;

        boost::asio::async_read_until(*serial_port_, read_buffer_, "\n",
                                      [this](const boost::system::error_code &ec, std::size_t /*bytes_transferred*/)
                                      {
                                          if (!ec)
                                          {
                                              std::istream is(&read_buffer_);
                                              std::string line;
                                              std::getline(is, line);

                                              // GRBL sends "ok" when it finishes parsing a command
                                              if (line.find("ok") != std::string::npos)
                                              {
                                                  arduino_ready_ = true;
                                              }
                                              async_read_ok(); // Keep listening
                                          }
                                      });
    }
} // namespace arctos_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arctos_control::ArctosSerialHardwareInterface, hardware_interface::SystemInterface)