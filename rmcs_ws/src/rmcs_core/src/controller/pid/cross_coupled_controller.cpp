#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::pid {

class CrossCoupledController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CrossCoupledController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        auto parameter_setpoint = get_parameter("setpoint");
        if (parameter_setpoint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            setpoint_immediate_value_ = parameter_setpoint.as_double();
            setpoint_.bind_directly(setpoint_immediate_value_);
        } else {
            register_input(parameter_setpoint.as_string(), setpoint_);
        }

        sync_compensation_     = get_parameter("sync_compensation").as_double();
        integral_compensation_ = get_parameter("integral_compensation").as_double();
        precision_             = get_parameter("precision").as_double();

        register_input(get_parameter("A-measurement").as_string(), A_measurement_);
        register_input(get_parameter("B-measurement").as_string(), B_measurement_);

        register_output(get_parameter("A-control_error").as_string(), A_output_error_);
        register_output(get_parameter("B-control_error").as_string(), B_output_error_);
    }

    void update() override {
        double weighted_relative_error, A_error, B_error;

        A_error = *setpoint_ - *A_measurement_;
        B_error = *setpoint_ - *B_measurement_;

        if (abs(A_error - B_error) < precision_) {
            weighted_relative_error = 0;
        } else {
            weighted_relative_error = (*A_measurement_ - *B_measurement_) * sync_compensation_;
        }

        weighted_error_integral_ += weighted_relative_error * integral_compensation_;

        *A_output_error_ = A_error - weighted_relative_error - weighted_error_integral_;
        *B_output_error_ = B_error + weighted_relative_error + weighted_error_integral_;
    }

private:
    double setpoint_immediate_value_;

    double sync_compensation_;
    double integral_compensation_;
    double weighted_error_integral_ = 0;
    double precision_               = 0.01;

    InputInterface<double> setpoint_;
    InputInterface<double> A_measurement_, B_measurement_;
    OutputInterface<double> A_output_error_, B_output_error_;
    OutputInterface<double> filtered_motor1_value_, filtered_motor2_value_;
};
} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::CrossCoupledController, rmcs_executor::Component)