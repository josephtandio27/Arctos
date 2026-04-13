#include "arctos_hardware_interface/arctos_serial_hardware_interface.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm> // for std::transform

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
        // Initialize vectors for 6 joints + 1 gripper
        hw_commands_.resize(params.hardware_info.joints.size(), 0.0);
        hw_states_.resize(params.hardware_info.joints.size(), 0.0);
        last_sent_commands_ = hw_commands_;

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
        std::string abs_motion_param = params.hardware_info.hardware_parameters.at("abs_motion");
        std::string lower_abs = abs_motion_param;
        std::transform(lower_abs.begin(), lower_abs.end(), lower_abs.begin(), ::tolower);
        if (lower_abs == "true" || lower_abs == "1")
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
            serial_port_->open(port_name_);
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(port_baud_rate_));

            // Start the IO service and ROS executor in a background thread
            // This ensures subscribers and "ok" listeners work even when INACTIVE
            if (service_thread_.joinable())
                return CallbackReturn::SUCCESS; // Prevent double start

            executor_.add_node(node_);
            thread_running_ = true;
            service_thread_ = std::thread([this]()
                                          {
                // This loop runs for the lifetime of the hardware interface
                while (rclcpp::ok() && thread_running_) {
                    io_service_.poll();
                    io_service_.reset();
                    executor_.spin_some();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } });

            // Initialize Subscribers here
            setup_subscribers();

            // Disable motors using sleep command
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (serial_port_ && serial_port_->is_open())
            {
                std::string sleep_cmd = "$SLP\n";
                boost::asio::write(*serial_port_, boost::asio::buffer(sleep_cmd));
            }

            RCLCPP_INFO(node_->get_logger(), "Serial port opened and background thread started.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Configure failed: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            // Start your "ok" token listener
            arduino_ready_.store(true);
            async_read_ok();

            // Set current axes as zero
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (serial_port_ && serial_port_->is_open())
            {
                std::string reset_cmd = "G92 X0.0 Y0.0 Z0.0 A0.0 B0.0 C0.0\n";
                boost::asio::write(*serial_port_, boost::asio::buffer(reset_cmd));
            }

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
        arduino_ready_ = false; // Stop the write() loop from sending G1
        RCLCPP_INFO(node_->get_logger(), "Movement disabled, but serial remains open for config.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosSerialHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardware"), "Cleaning up Arctos serial resources...");

        try
        {
            thread_running_ = false;
            // Stop the Boost IO service
            // This cancels any pending async_read_until ("ok" listener)
            if (!io_service_.stopped())
            {
                io_service_.stop();
            }
            if (service_thread_.joinable())
            {
                // You might need a flag to stop the while() loop in the thread
                service_thread_.join();
            }

            // Close and destroy the serial port object
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

            if (info_.joints[i].name == "Left_jaw_joint")
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_state_));
            }
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

    hardware_interface::return_type ArctosSerialHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        double dt = period.seconds();
        // OPEN LOOP: Since we have no encoders, we assume the robot reached the command.
        for (uint i = 0; i < hw_states_.size(); i++)
        {
            hw_states_[i] = hw_commands_[i];

            if (info_.joints[i].name == "Left_jaw_joint" && dt > 0.0)
            {
                gripper_velocity_state_ = (hw_states_[i] - last_gripper_position_) / dt;
                last_gripper_position_ = hw_states_[i];
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArctosSerialHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
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
            std::lock_guard<std::mutex> lock(serial_mutex_);
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

    void ArctosSerialHardwareInterface::setup_subscribers()
    {
        // Set the feed rate via subscriber:
        // ros2 topic pub --once /set_feed_rate std_msgs/msg/Int32 "{data: 1200}"
        feed_rate_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "set_feed_rate",
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->feed_rate_.store(msg->data);
                RCLCPP_INFO(node_->get_logger(), "Feed rate updated to: %d", this->feed_rate_.load());
            });

        // Send Raw GRBL Commands ($xx=val and any G-code)):
        // ros2 topic pub --once /send_grbl_command std_msgs/msg/String "{data: '$110=500'}"
        // ($SLP) to sleep and disable motors
        grbl_config_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "send_grbl_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                std::string cmd = msg->data;
                std::string upper_cmd = cmd;
                std::transform(upper_cmd.begin(), upper_cmd.end(), upper_cmd.begin(), ::toupper);

                if (upper_cmd.find("G92") != std::string::npos)
                {
                    std::stringstream ss(upper_cmd);
                    std::string token;
                    std::map<char, int> axis_map = {{'X', 0}, {'Y', 1}, {'Z', 2}, {'A', 3}, {'B', 4}, {'C', 5}};

                    while (ss >> token)
                    {
                        char axis = token[0];
                        // Check if this token is an axis letter (e.g., "X")
                        if (axis_map.count(axis))
                        {
                            double degrees;
                            // Handle "X0.0" vs "X 0.0"
                            if (token.size() > 1)
                            {
                                // Value is attached: "X0.5"
                                degrees = std::stod(token.substr(1));
                            }
                            else
                            {
                                // Value is next token: "X 0.5"
                                if (ss >> degrees)
                                { /* success */
                                }
                            }

                            int idx = axis_map[axis];
                            // CONVERT DEGREES (Hardware) -> RADIANS (ROS)
                            double radians = degrees * (M_PI / 180.0);

                            hw_states_[idx] = radians;
                            hw_commands_[idx] = radians;
                            last_sent_commands_[idx] = radians;
                        }
                    }
                }

                // Send to serial port
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (serial_port_ && serial_port_->is_open())
                {
                    std::string final_cmd = cmd + "\n";
                    boost::asio::write(*serial_port_, boost::asio::buffer(final_cmd));
                    RCLCPP_INFO(node_->get_logger(), "Serial Sent: %s", cmd.c_str());
                }
            });
    }
} // namespace arctos_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arctos_control::ArctosSerialHardwareInterface, hardware_interface::SystemInterface)