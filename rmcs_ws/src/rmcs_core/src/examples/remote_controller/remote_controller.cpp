#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::example {

#define PI 3.14159265358979323846
class RemoteControllerExample
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    RemoteControllerExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/remote/joystick/left", remote_left_joystic_);
        register_input("/remote/joystick/right", remote_right_joystic_);
        register_input("/remote/switch/left", remote_left_switch_);
        register_input("/remote/switch/right", remote_right_switch_);

        // register_input(
        //     "/example/m2006_no_1/remote_control_torque", m2006_no_1_remote_control_torque);
        // register_input(
        //     "/example/m2006_no_2/remote_control_torque", m2006_no_2_remote_control_torque);

        register_input("/example/gantry/gantry_now_pitch", now_pitch_);
        register_input("/example/gantry/gantry_now_roll", now_roll_);

        register_input("/example/m2006_no_1/control_m2006_no_1_pitch", m2006_no_1_control_pitch_);
        register_input("/example/m2006_no_2/control_angle", m2006_no_2_control_angle_);

        // register_output("/example/m2006_no_1/control_torque", m2006_no_1_control_torque);
        // register_output("/example/m2006_no_2/control_torque", m2006_no_2_control_torque);

        register_output("/example/m2006_no_1/control_velocity", m2006_no_1_control_velocity_);
        register_output("/example/m2006_no_2/control_velocity", m2006_no_2_control_velocity_);
        register_output("/example/gantry/control_pitch", motor_control_pitch_);
    }

    // void Control_Motor_Torque() {
    //     *m2006_no_1_control_torque = *m2006_no_1_remote_control_torque;
    //     *m2006_no_2_control_torque = *m2006_no_2_remote_control_torque;
    // }

    void Stop_Motor() {
        *m2006_no_1_control_velocity_ = 0.0;
        *m2006_no_2_control_velocity_ = 0.0;
    }

    double Deg_to_Rad(double deg) {
        double rad;
        rad = deg * PI / 180.0;
        return rad;
    }

    void Control_Gantry_Pitch(double target_pitch) {
        *motor_control_pitch_ = Deg_to_Rad(target_pitch);
        *m2006_no_1_control_velocity_ = *m2006_no_1_control_pitch_;
        *m2006_no_2_control_velocity_ = *m2006_no_2_control_angle_ + angle_error_;
        // Control_Motor_Torque();
    }

    void Gantry_Balance() {

        if ((*now_roll_) > 1E-6) {
            angle_error_ -= get_parameter("angle_error_ki").as_double();
        } else if ((*now_roll_) < -1E-6) {
            angle_error_ += get_parameter("angle_error_ki").as_double();
        }
        control_angle_error_ = angle_error_ * get_parameter("control_angle_error_kp").as_double();
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN
                || *remote_right_switch_ == Switch::UNKNOWN)) {
            // Stop_Motor();
            //  stop all !!
        } else if (*remote_left_switch_ == Switch::DOWN) {
            if (*remote_right_switch_ == Switch::MIDDLE) {
                *m2006_no_1_control_velocity_ = -20.0 * remote_left_joystic_->x();
                *m2006_no_2_control_velocity_ = -20.0 * remote_right_joystic_->x();
                // Control_Motor_Torque();
            }

        } else if (*remote_left_switch_ == Switch::MIDDLE) {
            if (*remote_right_switch_ == Switch::MIDDLE) {
                // Control_Gantry_Pitch(20.0);

            } else if (*remote_right_switch_ == Switch::UP) {
                *m2006_no_1_control_velocity_ = -(*m2006_no_1_control_pitch_);
                *m2006_no_2_control_velocity_ = *m2006_no_2_control_angle_ + control_angle_error_;
            }
        } else if (*remote_left_switch_ == Switch::UP) {
            if (*remote_right_switch_ == Switch::UP) {
                *m2006_no_1_control_velocity_ = -20.0 * remote_left_joystic_->x();
                *m2006_no_2_control_velocity_ = *m2006_no_2_control_angle_ + control_angle_error_;
                // Gantry_Balance();
                // *m2006_no_2_control_velocity_ = *m2006_no_2_control_angle_ +
                // control_angle_error_; Control_Motor_Torque();
            } else if (*remote_right_switch_ == Switch::MIDDLE) {
                Gantry_Balance();
                *m2006_no_2_control_velocity_ = *m2006_no_2_control_angle_ + control_angle_error_;
            }
        }
    }

private:
    rclcpp::Logger logger_;
    double angle_error_;
    double control_angle_error_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;

    // InputInterface<double> m2006_no_1_remote_control_torque;
    // InputInterface<double> m2006_no_2_remote_control_torque;

    InputInterface<double> now_pitch_;
    InputInterface<double> now_roll_;

    InputInterface<double> m2006_no_1_control_pitch_;
    InputInterface<double> m2006_no_2_control_angle_;

    // OutputInterface<double> m2006_no_1_control_torque;
    // OutputInterface<double> m2006_no_2_control_torque;

    OutputInterface<double> m2006_no_1_control_velocity_;
    OutputInterface<double> m2006_no_2_control_velocity_;
    OutputInterface<double> motor_control_pitch_;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::RemoteControllerExample, rmcs_executor::Component)