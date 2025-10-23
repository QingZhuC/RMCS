#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <cmath>

namespace rmcs_core::controller::mycomponents {
class MyInput
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MyInput()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/mycomponents/component_a/Sine", get_Sine_);
        register_input("/mycomponents/component_a/Cosine", get_Cosine_);
        register_output("/mycomponents/component_b/Sine_Add_Cosine", Sine_Add_Cosine_);
    }
    void update() override {
        *Sine_Add_Cosine_ = *get_Sine_ + *get_Cosine_;
        RCLCPP_INFO(get_logger(), "Sine_Add_Cosine: %lf", *Sine_Add_Cosine_);
    }

private:
    InputInterface<double> get_Sine_;
    InputInterface<double> get_Cosine_;
    OutputInterface<double> Sine_Add_Cosine_;
};
} // namespace rmcs_core::controller::mycomponents

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::mycomponents::MyInput, rmcs_executor::Component)