#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/type_support_decl.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dartlauncher {
/*
    so many limit to protect machine struct
    a better machine design will be relaxing
    dart explosion is as serious as Manba Out,but nobody see and care
*/
class DartGantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartGantryController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        pitch_init_angle_        = get_parameter("pitch_init_angle").as_double();
        pitch_lower_lower_limit_ = get_parameter("pitch_lower_limit").as_double();
        pitch_angle_upper_limit_ = get_parameter("pitch_upper_limit").as_double();
        pitch_velocity_limit_    = get_parameter("pitch_velocity_limit").as_double();

        register_input("/dart/device/imu_data", imu_data_);
        register_output("/dart/pitch_angle/current_angle", pitch_angle_current_value_, nan);

        register_input("/dart_guide/pitch_angle_setpoint", dart_guide_pitch_, false);
        register_output("/dart/pitch_angle/setpoint", pitch_angle_setpoint_);

        register_input("/dart/pitch_angle/pid_output_velocity", dart_pitch_expected_velocity_, false);
        register_output("/dart/pitch_angle/control_velocity", dart_pitch_control_velocity_, nan);

        register_input("/dart_guide/pitch_angle_lock", angle_lock_, false);

        launch_time_ = std::chrono::steady_clock::now();
    }

    void update() override {
        if (!imu_data_stable_) {
            auto delta_time_ =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - launch_time_);
            if (delta_time_.count() > 60) {
                RCLCPP_INFO(logger_, "imu_ready");
                *pitch_angle_setpoint_ = pitch_init_angle_;
                imu_data_stable_       = true;
            } else {
                return;
            }
        }

        if (*angle_lock_) {
            *dart_pitch_control_velocity_ = nan;
            RCLCPP_INFO(logger_, "angle_locked");
            return;
        }

        auto pitch_angle_value_ = calc_pitch_angle();
        if (pitch_angle_value_ > pitch_lower_lower_limit_ - 10 && pitch_angle_value_ < pitch_angle_upper_limit_ + 10) {
            *pitch_angle_current_value_ = pitch_angle_value_;

        } else if (
            180 - pitch_angle_value_ > pitch_lower_lower_limit_ - 10
            && 180 - pitch_angle_value_ < pitch_angle_upper_limit_ + 10) {
            // TODO: 这个仅临时使用,解决一下意外出现的万向锁，后续换成四元数
            *pitch_angle_current_value_ = 180 - pitch_angle_value_;

        } else {
            // be careful mad imu !
            *dart_pitch_control_velocity_ = nan;
            RCLCPP_INFO(logger_, "imu_data abnormal,imu_pitch:%lf", pitch_angle_value_);
            return;
        }

        // protect
        *pitch_angle_setpoint_ =
            std::max(pitch_lower_lower_limit_, std::min(*dart_guide_pitch_, pitch_angle_upper_limit_));
        *dart_pitch_control_velocity_ =
            std::max(-pitch_velocity_limit_, std::min(*dart_pitch_expected_velocity_, pitch_velocity_limit_));

        //
        RCLCPP_INFO(logger_, "current_pitch:%lf,setpoint:%lf", *pitch_angle_current_value_, *pitch_angle_setpoint_);
    }

private:
    double calc_pitch_angle() {
        auto dart_imu_pose             = imu_data_->normalized();
        Eigen::Matrix3d rotationMatrix = dart_imu_pose.toRotationMatrix();
        Eigen::Vector3d rpy_angles     = rotationMatrix.eulerAngles(2, 0, 1);
        double pitch                   = 180.0 + rpy_angles[1] * 180.0 / M_PI;
        return pitch;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    bool imu_data_stable_ = false;
    std::chrono::steady_clock::time_point launch_time_;
    InputInterface<Eigen::Quaterniond> imu_data_;
    OutputInterface<double> pitch_angle_current_value_;

    InputInterface<double> dart_guide_pitch_;
    OutputInterface<double> pitch_angle_setpoint_;

    InputInterface<double> dart_pitch_expected_velocity_;
    OutputInterface<double> dart_pitch_control_velocity_;

    InputInterface<bool> angle_lock_; // from dart_guide, fire stage angle should lock

    double pitch_init_angle_, pitch_angle_upper_limit_, pitch_lower_lower_limit_, pitch_velocity_limit_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartGantryController, rmcs_executor::Component)
