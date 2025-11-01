#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "librmcs/client/cboard.hpp"
#include "librmcs/device/bmi088.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class SingleCBoardExample
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {

public:
    SingleCBoardExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , balance_flag_(static_cast<uint8_t>(get_parameter("balance_flag").as_int()))
        , robot_command_(
              create_partner_component<RoboCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , bmi088_(1000, 0.2, 0.0)
        , M2006_NO_1_(*this, *robot_command_, "/example/m2006_no_1")
        , M2006_NO_2_(*this, *robot_command_, "/example/m2006_no_2")
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        register_output("/example/gantry/gantry_now_pitch", gantry_pitch_);
        register_output("/example/gantry/gantry_now_roll", gantry_roll_);
        register_output("/example/tf", tf_);

        M2006_NO_1_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle());
        M2006_NO_2_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle());

        bmi088_.set_coordinate_mapping([](double x, double y, double z) {
            // Get the mapping with the following code.
            // The rotation angle must be an exact multiple of 90 degrees, otherwise use a matrix.

            // Eigen::AngleAxisd pitch_link_to_imu_link{
            //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
            // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
            // std::cout << mapping << std::endl;

            return std::make_tuple(-y, x, z);
        });
    }

    ~SingleCBoardExample() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        update_imu();
        gantry_balance(balance_flag_);
        dr16_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = M2006_NO_1_.generate_command();
        can_commands[1] = M2006_NO_2_.generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        M2006_NO_1_.update_status();
        M2006_NO_2_.update_status();
    }

    void update_imu() {
        bmi088_.update_status();
        Eigen::Quaterniond gimbal_imu_pose{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *gantry_roll_ = bmi088_.ax();
        *gantry_pitch_ = bmi088_.ay();

        RCLCPP_INFO(logger_, "Gantry Roll: %.3f", *gantry_roll_);
        // RCLCPP_INFO(logger_, "Gantry Pitch: %.3f", *gantry_pitch_);
    }

    void gantry_balance(uint8_t flag) {
        // 实现一个简单的初始调平，当然以后也随时可以用
        if (flag) {
            M2006_NO_1_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M2006}.set_encoder_zero_point(
                    static_cast<int>(*gantry_roll_ / 360.0 * 8192)));
        } else {
        }
    }

protected:
    // 关于这里，如果你只有一个声明而没有实现，像我注释掉的内容一样，会报错

    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            M2006_NO_1_.store_status(can_data);
        } else if (can_id == 0x202) {
            M2006_NO_2_.store_status(can_data);
        }
    }

    // void can2_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        bmi088_.store_accelerometer_status(x, y, z);
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        bmi088_.store_gyroscope_status(x, y, z);
    }

private:
    rclcpp::Logger logger_;

    uint8_t balance_flag_;

    class RoboCommand : public rmcs_executor::Component {
    public:
        explicit RoboCommand(SingleCBoardExample& robot)
            : robot_(robot) {}

        void update() override { robot_.command_update(); }

        SingleCBoardExample& robot_;
    };
    std::shared_ptr<RoboCommand> robot_command_;

    device::Dr16 dr16_;

    device::Bmi088 bmi088_;

    device::DjiMotor M2006_NO_1_;
    device::DjiMotor M2006_NO_2_;

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<double> gantry_roll_;
    OutputInterface<double> gantry_pitch_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SingleCBoardExample, rmcs_executor::Component)