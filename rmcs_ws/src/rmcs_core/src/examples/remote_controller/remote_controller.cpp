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

        register_output("/example/m2006_mo_1/control_velocity", motor_control_velocity_);
        register_output("/example/gantry/pitch", motor_control_pitch_);
    }

    double Deg_to_Rad(double deg)
    {
        double rad;
        rad = deg * PI / 180.0;
        return rad;
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN || *remote_right_switch_ == Switch::UNKNOWN)) {
            // stop all !!
        } 
        else if(*remote_left_switch_ == Switch::MIDDLE)
        {
            if(*remote_right_switch_ == Switch::DOWN)
            {
                *motor_control_pitch_ = Deg_to_Rad(20.0);
            }
            else if(*remote_right_switch_ == Switch::MIDDLE)
            {
                *motor_control_pitch_ = Deg_to_Rad(30.0);
            }
            else if(*remote_right_switch_ == Switch::UP)
            {
                *motor_control_pitch_ = Deg_to_Rad(40.0);
            }
        }
        else if(*remote_left_switch_ == Switch::UP && *remote_right_switch_ == Switch::UP)
        {
            *motor_control_velocity_ = remote_left_joystic_->y() * 20.0;
        }
    }

private:
    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;

    OutputInterface<double> motor_control_velocity_;
    OutputInterface<double> motor_control_pitch_;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::RemoteControllerExample, rmcs_executor::Component)