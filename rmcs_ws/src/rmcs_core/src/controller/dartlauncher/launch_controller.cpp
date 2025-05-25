#include <chrono>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::dartlauncher {

class DartLaunchControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        auto first_friction_parameter = get_parameter("first_velocity");
        if (first_friction_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            first_fric_working_velocity_.make_and_bind_directly(first_friction_parameter.as_double());
        } else {
            register_input(first_friction_parameter.as_string(), first_fric_working_velocity_);
        }

        auto second_friction_parameter = get_parameter("second_velocity");
        if (second_friction_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            second_fric_working_velocity_.make_and_bind_directly(second_friction_parameter.as_double());
        } else {
            register_input(second_friction_parameter.as_string(), second_fric_working_velocity_);
        }

        timeout_limit_ = std::chrono::milliseconds(get_parameter("timeout_limit_ms").as_int());

        register_output("/dart/first_friction/control_velocity", first_fric_control_velocity_, nan);
        register_output("/dart/second_friction/control_velocity", second_fric_control_velocity_, nan);

        register_input("/dart/first_right_friction/velocity", first_fric_current_velocity_, false);
        register_input("/dart/second_right_friction/velocity", second_fric_current_velocity_, false);

        register_input("/dart/conveyor/velocity", conveyor_current_velocity_, false);
        register_output("/dart/conveyor/control_velocity", conveyor_control_velocity_, nan);

        register_output("/dart/launch_count", launch_count_, 0);
        register_input("/dart_guide/guide_ready", guide_ready_);
        register_input("/dart_guide/stop_all", stop_all_);

        launch_time_ = std::chrono::steady_clock::now();
    }

    void update() override {
        if (*stop_all_) {
            *conveyor_control_velocity_    = nan;
            *first_fric_control_velocity_  = nan;
            *second_fric_control_velocity_ = nan;
            return;
        }

        if (*guide_ready_) {
            *first_fric_control_velocity_  = *first_fric_working_velocity_;
            *second_fric_control_velocity_ = *second_fric_working_velocity_;
            stop_count_                    = 0;
        } else {
            if (abs(*first_fric_current_velocity_) <= 5 || stop_count_ > 1000) {
                *first_fric_control_velocity_  = nan;
                *second_fric_control_velocity_ = nan;
            } else {
                *first_fric_control_velocity_  = 0.0;
                *second_fric_control_velocity_ = 0.0;
                stop_count_++;
                // be careful mad motor
            }
        }

        if (*launch_count_ == 0) {
            auto current_timr = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(current_timr - launch_time_).count() < 5) {
                *conveyor_control_velocity_ = conveyor_down_velocity_ / 2;
                return;
            } else {
                *conveyor_control_velocity_ = nan;
                launch_ready_               = true;
                conveyor_direction_         = 0;
            }
        }

        dart_filling();
        *launch_count_ = dart_launch_count_;
    }

private:
    void dart_filling() {
        if (abs(*conveyor_current_velocity_) > 50) {
            push_block_moving_ = true;
        }

        if (launch_ready_) {
            if (*guide_ready_) {
                *conveyor_control_velocity_ = conveyor_up_velocity_;
                conveyor_direction_         = 1;
            } else {
                push_block_moving_ = false;
            }
        }

        if (conveyor_direction_ > 0) {
            if (conveyor_direction_ > 0 && push_block_moving_ && *conveyor_current_velocity_ == 0) {
                launch_ready_       = false;
                dart_launch_count_  = dart_launch_count_ + 1;
                conveyor_direction_ = -1;
                push_block_moving_  = false;
            }
        } // 换向

        if (conveyor_direction_ < 0) {
            *conveyor_control_velocity_ = conveyor_down_velocity_;
            if (push_block_moving_ && *conveyor_current_velocity_ == 0) {
                *conveyor_control_velocity_ = nan;
                push_block_moving_          = false;
                launch_ready_               = true;
                conveyor_direction_         = 0;
            }
        } // 下行与准备

        if (conveyor_direction_ == 0) {
            *conveyor_control_velocity_ = nan;
        }

        // TODO: if timeout

        // RCLCPP_INFO(
        //     logger_, "control:%lf,dir:%d,stable:%d,launch_ready:%d", *conveyor_control_velocity_,
        //     conveyor_direction_, push_block_moving_ ? 1 : 0, launch_ready_ ? 1 : 0);
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    int stop_count_             = 0;

    InputInterface<double> first_fric_working_velocity_, second_fric_working_velocity_;
    InputInterface<double> first_fric_current_velocity_, second_fric_current_velocity_;
    OutputInterface<double> first_fric_control_velocity_, second_fric_control_velocity_;

    int dart_launch_count_ = 0;
    bool launch_ready_     = false;

    InputInterface<double> conveyor_current_velocity_;
    OutputInterface<double> conveyor_control_velocity_;
    OutputInterface<int> launch_count_;
    InputInterface<bool> guide_ready_;
    InputInterface<bool> stop_all_;

    int conveyor_direction_        = -1;
    bool push_block_moving_        = false;
    double conveyor_up_velocity_   = 200.0;
    double conveyor_down_velocity_ = -400.0;
    std::chrono::steady_clock::time_point launch_time_;
    std::chrono::milliseconds timeout_limit_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartLaunchControl, rmcs_executor::Component)
