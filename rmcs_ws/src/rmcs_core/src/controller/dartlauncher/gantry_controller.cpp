#include <algorithm>
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

        pitch_lower_lower_limit_ = get_parameter("pitch_lower_limit").as_double();
        pitch_angle_upper_limit_ = get_parameter("pitch_upper_limit").as_double();
        pitch_velocity_limit_    = get_parameter("pitch_velocity_limit").as_double();

        register_input("/dart/device/imu_data", imu_data_);
        register_input("/dart_guide/pitch_angle/setpoint", dart_guide_pitch_setpoint_);
        register_output("/dart/pitch_angle/error", pitch_angle_error_);
        register_output("/dart/pitch_angle/current_angle", pitch_angle_current_value_, nan);
        register_input("/dart_guide/guide_ready", dart_guide_ready_);
        register_input("/dart_guide/stop_all", stop_all_);

        launch_time_ = std::chrono::steady_clock::now();
    }

    void update() override {
        if (!imu_data_stable_) {
            auto delta_time_ =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - launch_time_);
            if (delta_time_.count() > 60) {
                RCLCPP_INFO(logger_, "imu_ready,current_angle:%lf", calc_pitch_angle());
                imu_data_stable_ = true;
            } else {
                *pitch_angle_error_ = nan;
                return;
            }
        }

        if (*stop_all_) {
            *pitch_angle_error_ = nan;
            return;
        }

        if (*dart_guide_ready_) {
            *pitch_angle_error_ = nan;
            // RCLCPP_INFO(logger_, "angle_locked");
            return;
        }

        auto pitch_angle_value_ = calc_pitch_angle();
        if (pitch_angle_value_ > pitch_lower_lower_limit_ - 5 && pitch_angle_value_ < pitch_angle_upper_limit_ + 5) {
            *pitch_angle_current_value_ = pitch_angle_value_;

        } else if (
            180 - pitch_angle_value_ > pitch_lower_lower_limit_ - 5
            && 180 - pitch_angle_value_ < pitch_angle_upper_limit_ + 5) {
            // TODO: 这个仅临时使用,解决一下意外出现的万向锁，后续换成四元数
            *pitch_angle_current_value_ = 180 - pitch_angle_value_;

        } else {
            // be careful mad imu !
            *pitch_angle_error_ = 0.0;
            RCLCPP_INFO(logger_, "imu_data abnormal,imu_pitch:%lf", pitch_angle_value_);
            return;
        }

        // limit error value to control velocity
        double pitch_error  = *dart_guide_pitch_setpoint_ - *pitch_angle_current_value_;
        *pitch_angle_error_ = std::max(-pitch_velocity_limit_, std::min(pitch_error, pitch_velocity_limit_));

        //
        // RCLCPP_INFO(
        //     logger_, "current:%lf,set:%lf,error:%lf", *pitch_angle_current_value_, *dart_guide_pitch_setpoint_,
        //     *pitch_angle_error_);
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
    InputInterface<double> dart_guide_pitch_setpoint_;
    OutputInterface<double> pitch_angle_error_;

    InputInterface<bool> dart_guide_ready_;
    InputInterface<bool> stop_all_;
    OutputInterface<double> pitch_angle_current_value_;

    double pitch_angle_upper_limit_, pitch_lower_lower_limit_, pitch_velocity_limit_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartGantryController, rmcs_executor::Component)
