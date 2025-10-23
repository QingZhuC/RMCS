#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <cmath>

namespace rmcs_core::controller::mycomponents {
class MyOutput
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MyOutput()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/mycomponents/component_a/Sine", Sine_);
        register_output("/mycomponents/component_a/Cosine", Cosine_);

        omega_sine_ = get_parameter("omega_sine").as_double();
        omega_cosine_ = get_parameter("omega_cosine").as_double();
    }
    void update() override {
        *Sine_ = sin(t * omega_sine_);
        *Cosine_ = cos(t * omega_cosine_);

        t += 0.001;
    }

private:
    OutputInterface<double> Sine_;
    OutputInterface<double> Cosine_;
    double omega_sine_;
    double omega_cosine_;
    double t = 0.0;
};
} // namespace rmcs_core::controller::mycomponents

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::mycomponents::MyOutput, rmcs_executor::Component)