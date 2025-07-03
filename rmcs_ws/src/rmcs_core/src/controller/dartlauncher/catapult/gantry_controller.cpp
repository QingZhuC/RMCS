#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::CatapultDartLauncher {

class GantryMotorsController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryMotorsController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/catapult/gantry/pitch_angle/control_velocity", setpoint_);

        sync_compensation_     = get_parameter("sync_compensation").as_double();
        integral_compensation_ = get_parameter("integral_compensation").as_double();
        precision_             = get_parameter("precision").as_double();

        register_input("/catapult/gantry/pitch_left/velocity", left_motor_measurement_);
        register_input("/catapult/gantry/pitch_right/velocity", right_motor_measurement_);

        register_output("/catapult/gantry/pitch_left/control_error", left_motor_control_output_);
        register_output("/catapult/gantry/pitch_right/control_error", right_motor_control_output_);
    }

    void update() override {
        double weighted_relative_error, left_motor_error, right_motor_error;

        left_motor_error  = *setpoint_ - *left_motor_measurement_;
        right_motor_error = *setpoint_ - *right_motor_measurement_;

        if (abs(left_motor_error - right_motor_error) < precision_) {
            weighted_relative_error = 0;
        } else {
            weighted_relative_error = (*left_motor_measurement_ - *right_motor_measurement_) * sync_compensation_;
        }

        weighted_error_integral_ += weighted_relative_error * integral_compensation_;

        *left_motor_control_output_  = left_motor_error - weighted_relative_error - weighted_error_integral_;
        *right_motor_control_output_ = right_motor_error + weighted_relative_error + weighted_error_integral_;
    }

private:
    double sync_compensation_;
    double integral_compensation_;
    double weighted_error_integral_ = 0;
    double precision_               = 0.01;

    InputInterface<double> setpoint_;
    InputInterface<double> left_motor_measurement_, right_motor_measurement_;
    OutputInterface<double> left_motor_control_output_, right_motor_control_output_;
    OutputInterface<double> filtered_motor1_value_, filtered_motor2_value_;
};
} // namespace rmcs_core::controller::CatapultDartLauncher

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::CatapultDartLauncher::GantryMotorsController, rmcs_executor::Component)